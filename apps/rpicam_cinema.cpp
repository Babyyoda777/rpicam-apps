/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * rpicam_cinema.cpp - Preview-capable RAW CinemaDNG sequence capture app.
 *
 * Performance Optimized:
 * - Single-File Write (32MB Buffer): Streams all frames to one 'movie.raw' file.
 * - Multi-Threaded Conversion: Extracts DNGs from 'movie.raw' in parallel.
 * - Memory Optimized: Aggressively frees RAM after writing to disk.
 */

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <optional>
#include <future>
#include <algorithm>

#include <libcamera/controls.h>

#include "core/rpicam_app.hpp"
#include "core/stream_info.hpp"
#include "core/cinema_options.hpp"
#include "core/buffer_sync.hpp"

#include "image/image.hpp"

using namespace std::chrono_literals;
using libcamera::Stream;
namespace fs = std::filesystem;

class RPiCamCinemaApp : public RPiCamApp
{
public:
	RPiCamCinemaApp() : RPiCamApp(std::make_unique<CinemaOptions>()) {}
	CinemaOptions *GetOptions() const { return static_cast<CinemaOptions *>(RPiCamApp::GetOptions()); }
};

struct FrameJob
{
	std::string filename; // Final DNG filename
	StreamInfo info;
	libcamera::ControlList metadata;
	std::vector<uint8_t> bytes; // Frame data (cleared after writing to disk)
	size_t size; // Size of the data (needed for read-back)
};

static void ensure_output_dirs(fs::path const &base, std::string const &reel, std::string const &clip)
{
	fs::path dir = base / reel / clip;
	std::error_code ec;
	fs::create_directories(dir, ec);
	if (ec)
		throw std::runtime_error("Failed to create output directory: " + dir.string() + " (" + ec.message() + ")");
}

static std::string make_frame_filename(fs::path const &base, std::string const &reel, std::string const &clip,
									  uint32_t frame_1_based)
{
	char name[32];
	std::snprintf(name, sizeof(name), "%06u.dng", frame_1_based);
	return (base / reel / clip / name).string();
}

class FrameQueue
{
public:
	explicit FrameQueue(unsigned int max_depth) : max_depth_(std::max(1u, max_depth)) {}

	bool try_push(FrameJob &&job)
	{
		std::lock_guard<std::mutex> lk(mutex_);
		if (stop_)
			return false;
		if (queue_.size() >= max_depth_)
			return false;
		queue_.emplace_back(std::move(job));
		cv_.notify_one();
		return true;
	}

	bool pop(FrameJob &out)
	{
		std::unique_lock<std::mutex> lk(mutex_);
		cv_.wait(lk, [&] { return stop_ || !queue_.empty(); });
		if (queue_.empty())
			return false;
		out = std::move(queue_.front());
		queue_.pop_front();
		return true;
	}

	void stop()
	{
		{
			std::lock_guard<std::mutex> lk(mutex_);
			stop_ = true;
		}
		cv_.notify_all();
	}

	size_t size() const
	{
		std::lock_guard<std::mutex> lk(mutex_);
		return queue_.size();
	}

private:
	mutable std::mutex mutex_;
	std::condition_variable cv_;
	std::deque<FrameJob> queue_;
	unsigned int max_depth_;
	bool stop_ = false;
};

struct FpsWindow
{
	std::deque<std::chrono::steady_clock::time_point> ts;
	std::chrono::milliseconds window { 2000 };

	void push(std::chrono::steady_clock::time_point t)
	{
		ts.push_back(t);
		while (!ts.empty() && (t - ts.front()) > window)
			ts.pop_front();
	}

	double fps() const
	{
		if (ts.size() < 2)
			return 0.0;
		double secs = std::chrono::duration<double>(ts.back() - ts.front()).count();
		if (secs <= 0.0)
			return 0.0;
		return (double)(ts.size() - 1) / secs;
	}
};

// Thread to write to a SINGLE file (sequential append)
static void writer_thread_func(FrameQueue &q, std::vector<FrameJob> &deferred_jobs, 
                               std::mutex &deferred_mutex, std::atomic<uint64_t> &written_counter,
                               std::string const &raw_path)
{
	// Open single large file for appending
	FILE* fp = fopen(raw_path.c_str(), "wb");
	if (!fp) {
		LOG_ERROR("CRITICAL: Failed to open raw container: " << raw_path);
		return;
	}

	// OPTIMIZATION: Use a large write buffer (32MB) to reduce syscalls.
	std::vector<char> write_buf(1024 * 1024 * 32);
	setvbuf(fp, write_buf.data(), _IOFBF, write_buf.size());

	FrameJob job;
	while (q.pop(job))
	{
		size_t w = fwrite(job.bytes.data(), 1, job.bytes.size(), fp);
		if (w != job.bytes.size()) {
			LOG_ERROR("Write error! Expected " << job.bytes.size() << " wrote " << w);
		}

		// Save the size for read-back
		job.size = job.bytes.size();
		
		// CRITICAL MEMORY FIX: Force deallocation of vector memory
		// std::vector::clear() does NOT free memory, only swap does.
		std::vector<uint8_t>().swap(job.bytes);

		{
			std::lock_guard<std::mutex> lk(deferred_mutex);
			deferred_jobs.push_back(std::move(job));
		}

		written_counter.fetch_add(1, std::memory_order_relaxed);
	}
	fclose(fp);
}

static void disable_auto_exposure(RPiCamCinemaApp &app)
{
	libcamera::ControlList cl;
	cl.set(libcamera::controls::AeEnable, false);
	app.SetControls(cl);
}

static void convert_deferred_frames(std::vector<FrameJob> &jobs, std::string const &cam_model, CinemaOptions const *options, std::string const &raw_path)
{
	std::cout << "\nStarting Post-Processing (Extracting " << jobs.size() << " DNGs from container)...\n";
	
	FILE* fp = fopen(raw_path.c_str(), "rb");
	if (!fp) {
		LOG_ERROR("Failed to open raw container for reading: " << raw_path);
		return;
	}

	// Increase read buffer for faster sequential reading
	std::vector<char> read_buf(1024 * 1024 * 16);
	setvbuf(fp, read_buf.data(), _IOFBF, read_buf.size());

	int count = 0;
	int total = jobs.size();

	// Parallel Processing setup
	unsigned int max_threads = std::thread::hardware_concurrency();
	if (max_threads > 1) max_threads -= 1; // Leave one thread for the main reader
	if (max_threads < 1) max_threads = 1;

	std::vector<std::future<void>> futures;
	std::cout << "Using " << max_threads << " worker threads for DNG conversion.\n";

	auto process_dng = [&](std::vector<uint8_t> data, FrameJob job_copy) {
		try {
			std::vector<libcamera::Span<uint8_t>> mem;
			mem.emplace_back(data.data(), data.size());
			dng_save(mem, job_copy.info, job_copy.metadata, job_copy.filename, cam_model, options);
		} catch (std::exception const &e) {
			LOG_ERROR("Error processing frame " << job_copy.filename << ": " << e.what());
		}
	};

	// We assume jobs are in the same order they were written (FIFO queue)
	for (auto &job : jobs)
	{
		count++;
		if (count % 10 == 0)
			std::cout << "Processing " << count << "/" << total << "\r" << std::flush;

		// Read data sequentially in main thread (IO bound)
		std::vector<uint8_t> data(job.size);
		size_t r = fread(data.data(), 1, job.size, fp);
		
		if (r != job.size) {
			LOG_ERROR("Read error on frame " << count << ". Expected " << job.size << " got " << r);
			break;
		}

		// Clean up completed futures
		futures.erase(std::remove_if(futures.begin(), futures.end(), 
			[](const std::future<void>& f) { 
				return f.wait_for(std::chrono::seconds(0)) == std::future_status::ready; 
			}), futures.end());

		// Throttling: if we have max_threads active, wait for one to finish
		if (futures.size() >= max_threads) {
			futures.front().wait();
			futures.erase(futures.begin());
		}

		// Launch conversion in background (CPU bound)
		// We copy 'job' (metadata) and move 'data'
		futures.push_back(std::async(std::launch::async, process_dng, std::move(data), job));
	}

	// Wait for all remaining tasks
	for (auto &f : futures)
		f.wait();

	fclose(fp);
	
	// Optional: Remove the big raw file after success
	// fs::remove(raw_path);
	std::cout << "\nPost-Processing Complete.\n";
}

static void event_loop(RPiCamCinemaApp &app)
{
	CinemaOptions *options = app.GetOptions();

	if (options->Get().output.empty())
		throw std::runtime_error("No output directory specified. Use -o <dir>.");

	fs::path output_base = options->Get().output;
	ensure_output_dirs(output_base, options->Reel(), options->Clip());

	// Path for the single massive raw file
	std::string big_raw_path = (output_base / options->Reel() / options->Clip() / "movie.raw").string();

	unsigned int still_flags = RPiCamApp::FLAG_STILL_RAW;

	app.OpenCamera();

	if (!options->Get().zsl)
		options->Set().zsl = true;
	app.ConfigureZsl(still_flags);

	app.StartCamera();

	// Force Framerate
	if (options->Get().framerate.value_or(0.0) > 0.0)
	{
		float fps = options->Get().framerate.value();
		int64_t frame_time_us = 1000000 / fps;
		LOG(1, "Forcing FrameDurationLimits to " << frame_time_us << "us for " << fps << " fps");
		libcamera::ControlList controls;
		controls.set(libcamera::controls::FrameDurationLimits, { frame_time_us, frame_time_us });
		app.SetControls(controls);
	}

	disable_auto_exposure(app);

	// Buffer depth: 250 frames minimum (~4GB RAM usage for 16MB frames)
	// We rely on RAM to absorb glitches.
	unsigned int q_depth = options->QueueDepth();
	if (q_depth < 250) q_depth = 250; 
	LOG(1, "Queue depth set to " << q_depth);

	FrameQueue queue(q_depth);
	std::atomic<uint64_t> written(0);
	
	std::vector<FrameJob> deferred_jobs;
	std::mutex deferred_mutex;

	// Start Single-File Writer
	std::thread writer(writer_thread_func, std::ref(queue), std::ref(deferred_jobs), 
	                   std::ref(deferred_mutex), std::ref(written), big_raw_path);

	uint64_t dropped = 0;
	uint64_t enq = 0;

	FpsWindow fps_cap_window;
	FpsWindow fps_write_window;
	
	auto start_time = std::chrono::steady_clock::now();
	auto last_log = start_time;
	
	// FIX: Explicitly use std::chrono::milliseconds for timeout
	auto timeout_val = options->Get().timeout; 

	uint64_t written_last = 0;
	uint32_t frame_counter = 1;

	for (;;)
	{
		RPiCamApp::Msg msg = app.Wait();

		if (msg.type == RPiCamApp::MsgType::Timeout)
		{
			LOG_ERROR("ERROR: Device timeout detected, attempting a restart!!!");
			app.StopCamera();
			app.StartCamera();
			disable_auto_exposure(app);
			continue;
		}
		if (msg.type == RPiCamApp::MsgType::Quit)
			break;
		
		// Manual Timeout Check (Fix for infinite loop and type mismatch)
		auto now = std::chrono::steady_clock::now();
		if (timeout_val.value.count() > 0 && (now - start_time) >= timeout_val.value) {
			LOG(1, "Timeout reached. Stopping capture.");
			break;
		}

		if (msg.type != RPiCamApp::MsgType::RequestComplete)
			throw std::runtime_error("unrecognised message!");

		CompletedRequestPtr &completed = std::get<CompletedRequestPtr>(msg.payload);

		fps_cap_window.push(now);

		if (!options->Get().nopreview && app.ViewfinderStream())
			app.ShowPreview(completed, app.ViewfinderStream());

		// Process RAW Frame
		Stream *stream = app.RawStream();
		if (!stream) throw std::runtime_error("Raw stream not found");

		// Prepare Job
		FrameJob job;
		job.info = app.GetStreamInfo(stream);
		job.metadata = completed->metadata;
		job.filename = make_frame_filename(output_base, options->Reel(), options->Clip(), frame_counter++);

		// Copy Data using BufferReadSync (Fix for Mmap error)
		BufferReadSync r(&app, completed->buffers[stream]);
		const std::vector<libcamera::Span<uint8_t>> mem = r.Get();
		
		if (!mem.empty()) {
			job.bytes.assign(mem[0].data(), mem[0].data() + mem[0].size());
			job.size = job.bytes.size();

			// Push to Queue
			if (queue.try_push(std::move(job)))
			{
				enq++;
			}
			else
			{
				dropped++;
				LOG(2, "Queue full! Dropped frame " << (frame_counter - 1));
			}
		}

		// Statistics
		if (now - last_log >= 2s)
		{
			uint64_t w_total = written.load(std::memory_order_relaxed);
			uint64_t w_diff = w_total - written_last;
			
			// Hack: push timestamp for every written frame to approximate write FPS
			for(uint64_t k=0; k<w_diff; k++) fps_write_window.push(now);
			written_last = w_total;

			double fps_cap = fps_cap_window.fps();
			double fps_wrt = fps_write_window.fps();
			
			LOG(1, "Cap: " << fps_cap << " fps, Wrt: " << fps_wrt << " fps, Q: " << queue.size() 
			    << "/" << q_depth << ", Drop: " << dropped << " Total: " << enq);

			last_log = now;
		}
	}

	auto end = std::chrono::steady_clock::now();
	std::cout << "\nStopping...\n";

	queue.stop();
	writer.join();

	double seconds = std::chrono::duration<double>(end - start_time).count();
	std::cout << "Captured " << enq << " frames in " << seconds << " seconds (" << (enq/seconds) << " fps)\n";
	std::cout << "Dropped " << dropped << " frames.\n";

	app.StopCamera();

	// Post-Processing Phase
	convert_deferred_frames(deferred_jobs, app.CameraModel(), options, big_raw_path);
}

int main(int argc, char *argv[])
{
	try
	{
		RPiCamCinemaApp app;
		CinemaOptions *options = app.GetOptions();
		if (options->Parse(argc, argv))
		{
			// if (options->verbose >= 2) options->Print();

			event_loop(app);
		}
	}
	catch (std::exception const &e)
	{
		LOG_ERROR("ERROR: " << e.what());
		return 1;
	}
	return 0;
}
