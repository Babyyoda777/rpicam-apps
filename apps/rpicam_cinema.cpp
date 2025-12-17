/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * rpicam_cinema.cpp - High Performance RAW Capture
 *
 * - Captures .raw files + metadata.json (No DNG conversion on Pi).
 * - Forces user-defined Framerate, Gain, and Shutter.
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
	std::string raw_filename; // Temporary RAW filename
	std::string dng_filename; // Intended DNG filename (for metadata)
	StreamInfo info;
	float exposure_ms;
	float gain;
	std::vector<uint8_t> bytes;
};

static void ensure_output_dirs(fs::path const &base, std::string const &reel, std::string const &clip)
{
	fs::path dir = base / reel / clip;
	std::error_code ec;
	fs::create_directories(dir, ec);
	if (ec)
		throw std::runtime_error("Failed to create output directory: " + dir.string() + " (" + ec.message() + ")");
}

static std::string make_filename(fs::path const &base, std::string const &reel, std::string const &clip,
									  uint32_t frame, const char* ext)
{
	char name[32];
	std::snprintf(name, sizeof(name), "%06u.%s", frame, ext);
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

// Writes RAW bytes to disk
static void writer_thread_func(FrameQueue &q, std::vector<FrameJob> &meta_jobs, 
                               std::mutex &meta_mutex, std::atomic<uint64_t> &written_counter)
{
	FrameJob job;
	while (q.pop(job))
	{
		try
		{
			std::ofstream raw_file(job.raw_filename, std::ios::binary);
			if (raw_file)
			{
				raw_file.write(reinterpret_cast<const char*>(job.bytes.data()), job.bytes.size());
				raw_file.close();
				written_counter.fetch_add(1, std::memory_order_relaxed);

				// Free memory
				job.bytes.clear();
				job.bytes.shrink_to_fit();

				// Save metadata for JSON export
				{
					std::lock_guard<std::mutex> lk(meta_mutex);
					meta_jobs.push_back(std::move(job));
				}
			}
		}
		catch (std::exception const &e)
		{
			std::cerr << "Write error: " << e.what() << std::endl;
		}
	}
}

static void save_metadata_json(std::vector<FrameJob> &jobs, fs::path const &output_dir)
{
	std::cout << "\nSaving metadata.json for " << jobs.size() << " frames..." << std::endl;
	fs::path json_path = output_dir / "metadata.json";
	std::ofstream f(json_path);
	
	f << "[\n";
	for (size_t i = 0; i < jobs.size(); ++i)
	{
		auto &j = jobs[i];
		fs::path raw_p(j.raw_filename);
		
		f << "  {\n";
		f << "    \"raw_file\": \"" << raw_p.filename().string() << "\",\n";
		f << "    \"width\": " << j.info.width << ",\n";
		f << "    \"height\": " << j.info.height << ",\n";
		f << "    \"stride\": " << j.info.stride << ",\n";
		f << "    \"exposure_ms\": " << j.exposure_ms << ",\n";
		f << "    \"analogue_gain\": " << j.gain << "\n";
		f << "  }" << (i < jobs.size() - 1 ? "," : "") << "\n";
	}
	f << "]\n";
	std::cout << "Done. Transfer the folder to Mac and run dng_convert.py" << std::endl;
}

static void disable_auto_exposure(RPiCamCinemaApp &app)
{
	libcamera::ControlList cl;
	cl.set(libcamera::controls::AeEnable, false);
	
	CinemaOptions *options = app.GetOptions();
	if (options->Get().gain > 0.0f)
		cl.set(libcamera::controls::AnalogueGain, options->Get().gain);
	if (options->Get().shutter)
		cl.set(libcamera::controls::ExposureTime, options->Get().shutter.get<std::chrono::microseconds>());

	app.SetControls(cl);
}

static void event_loop(RPiCamCinemaApp &app)
{
	CinemaOptions *options = app.GetOptions();
	fs::path output_base = options->Get().output;
	ensure_output_dirs(output_base, options->Reel(), options->Clip());
	fs::path clip_dir = output_base / options->Reel() / options->Clip();

	app.OpenCamera();
	if (!options->Get().zsl) options->Set().zsl = true;
	app.ConfigureZsl(RPiCamApp::FLAG_STILL_RAW);
	app.StartCamera();

	// Enforce Framerate
	if (options->Get().framerate.value_or(0.0) > 0.0)
	{
		int64_t frame_time_us = 1000000 / options->Get().framerate.value();
		libcamera::ControlList controls;
		controls.set(libcamera::controls::FrameDurationLimits, { frame_time_us, frame_time_us });
		app.SetControls(controls);
	}

	disable_auto_exposure(app);

	FrameQueue queue(200);
	std::atomic<uint64_t> written(0);
	std::vector<FrameJob> meta_jobs;
	std::mutex meta_mutex;
	std::thread writer(writer_thread_func, std::ref(queue), std::ref(meta_jobs), std::ref(meta_mutex), std::ref(written));

	uint64_t dropped = 0;
	uint32_t frame_counter = 1;
	auto start_time = std::chrono::steady_clock::now();
	auto last_log = start_time;

	for (;;)
	{
		RPiCamApp::Msg msg = app.Wait();
		if (msg.type == RPiCamApp::MsgType::Timeout) { app.StopCamera(); app.StartCamera(); disable_auto_exposure(app); continue; }
		if (msg.type == RPiCamApp::MsgType::Quit) break;
		
		CompletedRequestPtr &completed = std::get<CompletedRequestPtr>(msg.payload);
		if (!options->Get().nopreview && app.ViewfinderStream()) app.ShowPreview(completed, app.ViewfinderStream());

		Stream *raw = app.RawStream();
		StreamInfo info = app.GetStreamInfo(raw);
		BufferReadSync r(&app, completed->buffers[raw]);
		const std::vector<libcamera::Span<uint8_t>> mem = r.Get();

		// Calculate total size
		size_t total = 0;
		for(auto &s : mem) total += s.size();

		FrameJob job;
		job.info = info;
		job.raw_filename = make_filename(output_base, options->Reel(), options->Clip(), frame_counter, "raw");
		job.bytes.resize(total);
		
		size_t off = 0;
		for(auto &s : mem) { std::memcpy(job.bytes.data() + off, s.data(), s.size()); off += s.size(); }

		// Extract Metadata
		auto exp = completed->metadata.get(libcamera::controls::ExposureTime);
		job.exposure_ms = exp.value_or(0) / 1000.0f;
		auto gain = completed->metadata.get(libcamera::controls::AnalogueGain);
		job.gain = gain.value_or(1.0f);

		if (!queue.try_push(std::move(job))) dropped++;
		else frame_counter++;

		auto now = std::chrono::steady_clock::now();
		if (options->Get().timeout && (now - start_time) > options->Get().timeout.value) break;

		if ((now - last_log) >= 1s)
		{
			std::cout << "Frames: " << frame_counter << " Written: " << written << " Dropped: " << dropped << " Queue: " << queue.size() << std::endl;
			last_log = now;
		}
	}

	queue.stop();
	if (writer.joinable()) writer.join();
	
	save_metadata_json(meta_jobs, clip_dir);
}

int main(int argc, char *argv[])
{
	try { RPiCamCinemaApp app; if (app.GetOptions()->Parse(argc, argv)) event_loop(app); }
	catch (std::exception const &e) { std::cerr << "ERROR: " << e.what() << std::endl; return 1; }
	return 0;
}
