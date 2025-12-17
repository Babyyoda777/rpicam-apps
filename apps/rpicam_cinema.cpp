/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * rpicam_cinema.cpp - Preview-capable RAW CinemaDNG sequence capture app.
 *
 * Modified for High Performance:
 * - Uses Deferred Processing: Writes RAW bytes to disk during capture, converts to DNG after.
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

#include <libcamera/controls.h>

#include "core/rpicam_app.hpp"
#include "core/stream_info.hpp"
#include "core/cinema_options.hpp"

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
	std::string filename;     // Final DNG filename
	std::string raw_filename; // Temporary RAW filename
	StreamInfo info;
	libcamera::ControlList metadata;
	std::vector<uint8_t> bytes; // Empty after writing to temp raw file
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

static std::string make_raw_filename(fs::path const &base, std::string const &reel, std::string const &clip,
									  uint32_t frame_1_based)
{
	char name[32];
	std::snprintf(name, sizeof(name), "%06u.raw", frame_1_based);
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

// Thread to write raw bytes to disk as fast as possible
static void writer_thread_func(FrameQueue &q, std::vector<FrameJob> &deferred_jobs, 
                               std::mutex &deferred_mutex, std::atomic<uint64_t> &written_counter)
{
	FrameJob job;
	while (q.pop(job))
	{
		try
		{
			// Write raw bytes to temporary file
			std::ofstream raw_file(job.raw_filename, std::ios::binary);
			if (!raw_file)
				throw std::runtime_error("Failed to open raw file for writing");
			
			raw_file.write(reinterpret_cast<const char*>(job.bytes.data()), job.bytes.size());
			raw_file.close();

			// Clear memory but keep metadata for post-processing
			job.bytes.clear();
			job.bytes.shrink_to_fit();

			{
				std::lock_guard<std::mutex> lk(deferred_mutex);
				deferred_jobs.push_back(std::move(job));
			}

			written_counter.fetch_add(1, std::memory_order_relaxed);
		}
		catch (std::exception const &e)
		{
			LOG_ERROR("ERROR: RAW write failed for " << job.raw_filename << ": " << e.what());
		}
	}
}

static void disable_auto_exposure(RPiCamCinemaApp &app)
{
	libcamera::ControlList cl;
	cl.set(libcamera::controls::AeEnable, false);
	app.SetControls(cl);
}

static void convert_deferred_frames(std::vector<FrameJob> &jobs, std::string const &cam_model, CinemaOptions const *options)
{
	std::cout << "\nStarting Post-Processing (Converting " << jobs.size() << " frames to DNG)...\n";
	
	int count = 0;
	int total = jobs.size();

	for (auto &job : jobs)
	{
		count++;
		if (count % 10 == 0)
			std::cout << "Processing " << count << "/" << total << "\r" << std::flush;

		try 
		{
			// Read raw file back
			std::ifstream raw_file(job.raw_filename, std::ios::binary | std::ios::ate);
			if (!raw_file)
			{
				LOG_ERROR("Failed to open raw file: " << job.raw_filename);
				continue;
			}
			
			std::streamsize size = raw_file.tellg();
			raw_file.seekg(0, std::ios::beg);

			std::vector<uint8_t> data(size);
			if (!raw_file.read(reinterpret_cast<char*>(data.data()), size))
			{
				LOG_ERROR("Failed to read raw file: " << job.raw_filename);
				continue;
			}
			raw_file.close();

			// Construct memory span for dng_save
			std::vector<libcamera::Span<uint8_t>> mem;
			mem.emplace_back(data.data(), data.size());

			dng_save(mem, job.info, job.metadata, job.filename, cam_model, options);

			// Delete temp raw file
			fs::remove(job.raw_filename);
		}
		catch (std::exception const &e)
		{
			LOG_ERROR("Error processing frame " << job.filename << ": " << e.what());
		}
	}
	std::cout << "\nPost-Processing Complete.\n";
}

static void event_loop(RPiCamCinemaApp &app)
{
	CinemaOptions *options = app.GetOptions();

	if (options->Get().output.empty())
		throw std::runtime_error("No output directory specified. Use -o <dir>.");

	fs::path output_base = options->Get().output;
	ensure_output_dirs(output_base, options->Reel(), options->Clip());

	unsigned int still_flags = RPiCamApp::FLAG_STILL_RAW;

	app.OpenCamera();

	// Use ZSL to keep a running pipeline
	if (!options->Get().zsl)
		options->Set().zsl = true;
	app.ConfigureZsl(still_flags);

	app.StartCamera();

	// FIX: Force FrameDurationLimits to strictly enforce the requested framerate
	// Using options->Get().framerate as verified in previous errors
	if (options->Get().framerate.value_or(0.0) > 0.0)
	{
		float fps = options->Get().framerate.value();
		int64_t frame_time_us = 1000000 / fps;
		LOG(1, "Forcing FrameDurationLimits to " << frame_time_us << "us for " << fps << " fps");
		libcamera::ControlList controls;
		// Set min and max duration to the same value to lock framerate
		controls.set(libcamera::controls::FrameDurationLimits, { frame_time_us, frame_time_us });
		app.SetControls(controls);
	}

	disable_auto_exposure(app);

	// Large queue for buffering
	unsigned int q_depth = options->QueueDepth();
	if (q_depth < 200) q_depth = 200; // Buffer ~150-200 frames (2-3 GB RAM use)
	LOG(1, "Queue depth set to " << q_depth);

	FrameQueue queue(q_depth);
	std::atomic<uint64_t> written(0);
	
	// Store jobs to process after recording
	std::vector<FrameJob> deferred_jobs;
	std::mutex deferred_mutex;

	// Single high-speed writer thread for raw dumps
	std::thread writer(writer_thread_func, std::ref(queue), std::ref(deferred_jobs), 
	                   std::ref(deferred_mutex), std::ref(written));

	uint64_t dropped = 0;
	uint64_t enq = 0;

	FpsWindow fps_cap_window;
	FpsWindow fps_write_window;
	auto last_log = std::chrono::steady_clock::now();
	auto start = last_log;
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
		if (msg.type != RPiCamApp::MsgType::RequestComplete)
			throw std::runtime_error("unrecognised message!");

		CompletedRequestPtr &completed = std::get<CompletedRequestPtr>(msg.payload);

		auto now = std::chrono::steady_clock::now();
		fps_cap_window.push(now);

		if (!options->Get().nopreview && app.ViewfinderStream())
			app.ShowPreview(completed, app.ViewfinderStream());

		Stream *raw = app.RawStream();
		if (!raw)
			throw std::runtime_error("RAW stream not configured/enabled in this mode.");

		StreamInfo info = app.GetStreamInfo(raw);
		BufferReadSync r(&app, completed->buffers[raw]);
		const std::vector<libcamera::Span<uint8_t>> mem = r.Get();

		size_t total = 0;
		for (auto const &s : mem)
			total += s.size();

		FrameJob job;
		job.info = info;
		job.metadata = completed->metadata;
		job.filename = make_frame_filename(output_base, options->Reel(), options->Clip(), frame_counter);
		job.raw_filename = make_raw_filename(output_base, options->Reel(), options->Clip(), frame_counter);
		job.bytes.resize(total);

		size_t off = 0;
		for (auto const &s : mem)
		{
			std::memcpy(job.bytes.data() + off, s.data(), s.size());
			off += s.size();
		}

		if (!queue.try_push(std::move(job)))
		{
			dropped++;
		}
		else
		{
			enq++;
			frame_counter++;
		}

		// Update write stats
		uint64_t written_now = written.load();
		if (written_now > written_last)
		{
			for(uint64_t i=0; i < (written_now - written_last); i++)
				fps_write_window.push(now);
			written_last = written_now;
		}

		if (options->Get().timeout && (now - start) > options->Get().timeout.value)
		{
			app.StopCamera(); 
			break;
		}

		if ((now - last_log) >= 1s)
		{
			float exp_time = 0;
			float gain = 0;

			// SAFE METADATA ACCESS
			auto exp_val = completed->metadata.get(libcamera::controls::ExposureTime);
			if (exp_val) exp_time = *exp_val / 1000.0f;

			auto gain_val = completed->metadata.get(libcamera::controls::AnalogueGain);
			if (gain_val) gain = *gain_val;

			LOG(1, "#" << frame_counter << " (" << fps_cap_window.fps() << " fps) exp " 
			    << exp_time << " ms ag " << gain
			    << " fps_write~" << fps_write_window.fps() << " enq=" << enq 
			    << " written=" << written_now 
			    << " drop=" << dropped << " q=" << queue.size());

			last_log = now;
		}
	}

	// Stop queue and join writer
	queue.stop();
	if (writer.joinable())
		writer.join();

	// Post-process the frames
	if (!deferred_jobs.empty())
	{
		convert_deferred_frames(deferred_jobs, app.CameraModel(), options);
	}
}

int main(int argc, char *argv[])
{
	try
	{
		RPiCamCinemaApp app;
		CinemaOptions *options = app.GetOptions();
		if (options->Parse(argc, argv))
		{
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
