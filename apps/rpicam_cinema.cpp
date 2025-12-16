/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * rpicam_cinema.cpp - Preview-capable RAW CinemaDNG sequence capture app.
 *
 * Output layout (folder-per-clip):
 *   <output>/<reel>/<clip>/000000.dng
 *   <output>/<reel>/<clip>/000001.dng
 *   ...
 *
 * Always captures RAW, uses existing rpicam-apps DNG writer (dng_save).
 * Uses a writer thread and bounded queue to avoid stalling capture.
 */

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "core/rpicam_still_app.hpp"
#include "core/rpicam_app.hpp"
#include "core/stream_info.hpp"
#include "core/logging.hpp"

#include "core/cinema_options.hpp"
#include "image/image.hpp"

using namespace std::chrono_literals;
namespace fs = std::filesystem;

struct FrameJob
{
	std::string filename;
	StreamInfo info;
	libcamera::ControlList metadata;
	std::vector<uint8_t> bytes; // owned copy of the raw frame
};

static std::string make_frame_filename(fs::path const &base, std::string const &reel, std::string const &clip,
									  uint32_t frame)
{
	char name[32];
	std::snprintf(name, sizeof(name), "%06u.dng", frame);

	fs::path dir = base / reel / clip;
	return (dir / name).string();
}

static void ensure_output_dirs(fs::path const &base, std::string const &reel, std::string const &clip)
{
	fs::path dir = base / reel / clip;
	std::error_code ec;
	fs::create_directories(dir, ec);
	if (ec)
		throw std::runtime_error("Failed to create output directory: " + dir.string() + " (" + ec.message() + ")");
}

class FrameQueue
{
public:
	explicit FrameQueue(unsigned int max_depth) : max_depth_(std::max(1u, max_depth)) {}

	// returns false if dropped
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
	unsigned int max_depth_;
	mutable std::mutex mutex_;
	std::condition_variable cv_;
	std::deque<FrameJob> queue_;
	bool stop_ = false;
};

static void writer_thread_func(FrameQueue &q, std::atomic<bool> &done, std::string const &cam_model,
							   CinemaOptions const *options)
{
	uint64_t written = 0;
	while (!done.load())
	{
		FrameJob job;
		if (!q.pop(job))
		{
			// stop signalled
			break;
		}

		// Wrap the owned bytes in a Span for dng_save.
		std::vector<libcamera::Span<uint8_t>> mem;
		mem.emplace_back(job.bytes.data(), job.bytes.size());

		try
		{
			dng_save(mem, job.info, job.metadata, job.filename, cam_model, options);
			written++;
			if (options->Get().verbose >= 2 && (written % 25 == 0))
				LOG(2, "DNG writer: written " << written << " frames");
		}
		catch (std::exception const &e)
		{
			LOG_ERROR("ERROR: DNG write failed for " << job.filename << ": " << e.what());
		}
	}
}

static void event_loop(RPiCamStillApp &app, CinemaOptions *options)
{
	// Output base dir is options->Get().output (same as still).
	if (options->Get().output.empty())
		throw std::runtime_error("No output directory specified. Use -o <dir>.");

	fs::path output_base = options->Get().output;
	ensure_output_dirs(output_base, options->Reel(), options->Clip());

	// App setup.
	app.OpenCamera();

	// Configure preview/viewfinder.
	// rpicam-still typically configures viewfinder before still config.
	app.ConfigureViewfinder();

	// Configure still with RAW stream enabled.
	// Buffering hint helps reduce stalls.
	unsigned int still_flags = RPiCamApp::FLAG_STILL_RAW | RPiCamApp::FLAG_STILL_TRIPLE_BUFFER;
	app.ConfigureStill(still_flags);

	// Start camera/preview.
	app.StartCamera();

	FrameQueue queue(options->QueueDepth());
	std::atomic<bool> done(false);
	std::thread writer(writer_thread_func, std::ref(queue), std::ref(done), app.CameraModel(), options);

	uint64_t dropped = 0;
	uint64_t enqueued = 0;

	auto start_time = std::chrono::high_resolution_clock::now();
	auto last_log = start_time;

	// We will write one DNG per completed request (RAW stream).
	while (true)
	{
		RPiCamApp::Msg msg = app.Wait();

		if (msg.type == RPiCamApp::MsgType::Timeout)
		{
			LOG_ERROR("ERROR: Device timeout detected, attempting a restart!!!");
			app.StopCamera();
			app.StartCamera();
			continue;
		}
		if (msg.type != RPiCamApp::MsgType::RequestComplete)
			throw std::runtime_error("unrecognised message!");

		CompletedRequestPtr completed = std::get<CompletedRequestPtr>(msg.payload);

		// Show preview from viewfinder stream if enabled.
		// (RPiCamApp::ShowPreview is typically called by apps each completed request.)
		if (!options->Get().nopreview)
		{
			Stream *vf = app.ViewfinderStream();
			if (vf)
				app.ShowPreview(completed, vf);
		}

		Stream *raw = app.RawStream();
		if (!raw)
			throw std::runtime_error("RAW stream not configured (unexpected).");

		// Map RAW buffer and copy it immediately so we can return to capture quickly.
		StreamInfo info = app.GetStreamInfo(raw);
		BufferReadSync r(&app, completed->buffers[raw]);
		const std::vector<libcamera::Span<uint8_t>> mem = r.Get();

		// Concatenate planes (normally raw is single plane; handle generic case).
		size_t total = 0;
		for (auto const &s : mem)
			total += s.size();

		FrameJob job;
		job.info = info;
		job.metadata = completed->metadata;
		job.filename = make_frame_filename(output_base, options->Reel(), options->Clip(), options->Get().framestart);
		job.bytes.resize(total);

		size_t off = 0;
		for (auto const &s : mem)
		{
			std::memcpy(job.bytes.data() + off, s.data(), s.size());
			off += s.size();
		}

		bool pushed = queue.try_push(std::move(job));
		if (!pushed)
		{
			dropped++;
			if (options->Get().verbose >= 1 && (dropped % 25 == 1))
				LOG_ERROR("WARNING: DNG writer queue full, dropping frames. dropped=" << dropped
																					<< " queue=" << queue.size());
		}
		else
		{
			enqueued++;
			options->Set().framestart++;
			if (options->Get().wrap)
				options->Set().framestart %= options->Get().wrap;
		}

		// Timeout handling similar to rpicam apps.
		auto now = std::chrono::high_resolution_clock::now();
		if (options->Get().timeout && (now - start_time) > options->Get().timeout.value)
			break;

		// Periodic log.
		if (options->Get().verbose >= 1 && (now - last_log) > 2s)
		{
			double seconds = std::chrono::duration<double>(now - start_time).count();
			double fps_in = seconds > 0 ? (double)(enqueued + dropped) / seconds : 0;
			double fps_written = seconds > 0 ? (double)enqueued / seconds : 0; // approx; writer may lag
			LOG(1, "Frames: enq=" << enqueued << " drop=" << dropped << " fps_in=" << fps_in
								 << " fps_enq=" << fps_written << " q=" << queue.size());
			last_log = now;
		}
	}

	// Shutdown.
	app.StopCamera();
	done.store(true);
	queue.stop();
	if (writer.joinable())
		writer.join();
}

int main(int argc, char *argv[])
{
	try
	{
		auto opts = std::make_unique<CinemaOptions>();
		RPiCamStillApp app(std::move(opts));
		CinemaOptions *options = static_cast<CinemaOptions *>(app.GetOptions());

		if (options->Parse(argc, argv))
		{
			// Always RAW DNG sequence; force relevant still options.
			options->Set().raw = true;              // harmless, but keeps semantics aligned
			options->Set().encoding = "jpg";        // unused; we never write processed stills
			options->Set().denoise = "cdn_off";     // typical for raw workflows
			// keep preview default behavior (user can set --nopreview)

			if (options->Get().verbose >= 2)
				options->Get().Print();

			event_loop(app, options);
		}
	}
	catch (std::exception const &e)
	{
		LOG_ERROR("ERROR: *** " << e.what() << " ***");
		return -1;
	}
	return 0;
}
