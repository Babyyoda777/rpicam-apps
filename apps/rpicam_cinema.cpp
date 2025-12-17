/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * rpicam_cinema.cpp - Preview-capable RAW CinemaDNG sequence capture app.
 *
 * Output layout (folder-per-clip):
 *   <output>/<reel>/<clip>/000001.dng
 *   <output>/<reel>/<clip>/000002.dng
 *   ...
 *
 * Always captures RAW and uses existing dng_save() (image/dng.cpp).
 * Uses a writer thread + bounded queue to avoid stalling capture.
 *
 * Also:
 * - Disables auto exposure to prevent intermittent brightness/dark frames.
 * - Better FPS reporting via sliding window (capture) + written counter.
 */

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <filesystem>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

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
	std::string filename;
	StreamInfo info;
	libcamera::ControlList metadata;
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
	unsigned int max_depth_;
	mutable std::mutex mutex_;
	std::condition_variable cv_;
	std::deque<FrameJob> queue_;
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

static void writer_thread_func(FrameQueue &q, std::atomic<bool> &done, std::atomic<uint64_t> &written_counter,
							   std::string const &cam_model, CinemaOptions const *options)
{
	while (!done.load())
	{
		FrameJob job;
		if (!q.pop(job))
			break;

		std::vector<libcamera::Span<uint8_t>> mem;
		mem.emplace_back(job.bytes.data(), job.bytes.size());

		try
		{
			dng_save(mem, job.info, job.metadata, job.filename, cam_model, options);
			written_counter.fetch_add(1, std::memory_order_relaxed);
		}
		catch (std::exception const &e)
		{
			LOG_ERROR("ERROR: DNG write failed for " << job.filename << ": " << e.what());
		}
	}
}

static void disable_auto_exposure(RPiCamCinemaApp &app)
{
	libcamera::ControlList cl;
	cl.set(libcamera::controls::AeEnable, false);
	app.SetControls(cl);
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

	// Use ZSL to keep a running pipeline (best chance for preview+raw without core edits).
	if (!options->Get().zsl)
		options->Set().zsl = true;
	app.ConfigureZsl(still_flags);

	app.StartCamera();
	disable_auto_exposure(app);

	FrameQueue queue(options->QueueDepth());
	std::atomic<bool> done(false);
	std::atomic<uint64_t> written(0);
	std::thread writer(writer_thread_func, std::ref(queue), std::ref(done), std::ref(written), app.CameraModel(),
					   options);

	uint64_t dropped = 0;
	uint64_t enq = 0;

	FpsWindow fps_cap_window;
	auto last_log = std::chrono::steady_clock::now();
	auto start = last_log;
	uint64_t written_last = 0;

	uint32_t frame_counter = 1; // 1-based numbering as requested

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
			frame_counter++; // advance only when successfully enqueued (keeps sequence contiguous)
		}

		if (options->Get().timeout && (now - start) > options->Get().timeout.value)
			break;

		if (options->Get().verbose >= 1 && (now - last_log) > 1s)
		{
			uint64_t w = written.load(std::memory_order_relaxed);
			double secs = std::chrono::duration<double>(now - last_log).count();
			double fps_written_inst = secs > 0 ? (double)(w - written_last) / secs : 0.0;

			LOG(1, "FPS cap~" << fps_cap_window.fps() << " fps_write~" << fps_written_inst << " enq=" << enq
							 << " written=" << w << " drop=" << dropped << " q=" << queue.size());

			written_last = w;
			last_log = now;
		}
	}

	app.StopCamera();
	done.store(true);
	queue.stop();
	writer.join();
}

int main(int argc, char *argv[])
{
	try
	{
		RPiCamCinemaApp app;
		CinemaOptions *options = app.GetOptions();

		if (options->Parse(argc, argv))
		{
			options->Set().raw = true; // always raw

			if (options->Get().verbose >= 2)
				options->Get().Print();

			event_loop(app);
		}
	}
	catch (std::exception const &e)
	{
		LOG_ERROR("ERROR: *** " << e.what() << " ***");
		return -1;
	}
	return 0;
}
