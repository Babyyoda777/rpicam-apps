/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Test code by babyyoda777 (NOT VERIFIED TO WORK RELIABLY)
 *
 * rpicam_cinema.cpp - libcamera raw video (CinemaDNG) record app.
 */

#include <chrono>
#include <poll.h>
#include <signal.h>
#include <sys/signalfd.h>
#include <sys/stat.h>
#include <iostream>
#include <iomanip>
#include <sstream>

#include <libcamera/stream.h>
#include "core/rpicam_encoder.hpp" // Use encoder base class for video loop
#include "encoder/null_encoder.hpp" // Use NullEncoder to disable video processing
#include "output/output.hpp" // Use Output class for file handling
#include "image/image.hpp" // For dng_save function
#include "core/still_options.hpp" // For DNG metadata
#include "core/stream_info.hpp" // Correct location for StreamInfo structure

using namespace std::placeholders;
using libcamera::Stream;
using libcamera::StreamConfiguration;
using libcamera::Camera;

// --- Custom Application Class ---

class RPiCamCinema : public RPiCamEncoder
{
public:
	RPiCamCinema() : RPiCamEncoder() {}

protected:
	// Force the use of "null" encoder, as we are saving raw frames, not encoded video.
	void createEncoder() override { encoder_ = std::unique_ptr<Encoder>(new NullEncoder(GetOptions())); }
};

// --- Signal and Keypress Handling Functions ---

static int signal_received;
static void default_signal_handler(int signal_number)
{
	signal_received = signal_number;
	LOG(1, "Received signal " << signal_number);
}

static int get_key_or_signal(VideoOptions const *options_ptr, pollfd p[1])
{
	auto const &options = options_ptr->Get(); // The options struct reference
	int key = 0;
	if (signal_received == SIGINT)
		return 'x';
	if (options.keypress)
	{
		poll(p, 1, 0);
		if (p[0].revents & POLLIN)
		{
			char *user_string = nullptr;
			size_t len;
			[[maybe_unused]] size_t r = getline(&user_string, &len, stdin);
			key = user_string[0];
		}
	}
	if (options.signal)
	{
		if (signal_received == SIGUSR1)
			key = '\n';
		else if ((signal_received == SIGUSR2) || (signal_received == SIGPIPE))
			key = 'x';
		signal_received = 0;
	}
	return key;
}

// --- Main Application Event Loop ---

static void event_loop(RPiCamCinema &app)
{
	VideoOptions *options_ptr = app.GetOptions();
	auto const &options = options_ptr->Get(); // The options struct reference

	// Output is configured but is ignored since we are manually writing DNG files.
	std::unique_ptr<Output> output = std::unique_ptr<Output>(Output::Create(options_ptr));
	app.SetEncodeOutputReadyCallback(std::bind(&Output::OutputReady, output.get(), _1, _2, _3, _4));
	app.SetMetadataReadyCallback(std::bind(&Output::MetadataReady, output.get(), _1));

	// 1. Open Camera
	app.OpenCamera();
    
    // 2. Configure for RAW video streaming
	app.ConfigureVideo(RPiCamEncoder::FLAG_VIDEO_RAW);
	
	// 3. Start Encoder (NullEncoder) and Camera
	app.StartEncoder();
	app.StartCamera();
	auto start_time = std::chrono::high_resolution_clock::now();

	// Monitoring for keypresses and signals.
	signal(SIGUSR1, default_signal_handler);
	signal(SIGUSR2, default_signal_handler);
	signal(SIGINT, default_signal_handler);
	signal(SIGPIPE, default_signal_handler);
	pollfd p[1] = { { STDIN_FILENO, POLLIN, 0 } };

	LOG(1, "Starting CinemaDNG capture. Press 'x' or Ctrl+C to stop.");
	LOG(1, "WARNING: RAW video requires extremely fast storage!");

	for (unsigned int count = 0; ; count++)
	{
		RPiCamCinema::Msg msg = app.Wait();

		if (msg.type == RPiCamApp::MsgType::Timeout)
		{
			LOG_ERROR("ERROR: Device timeout detected, attempting a restart!!!");
			app.StopCamera();
			app.StartCamera();
			continue;
		}
		if (msg.type == RPiCamApp::MsgType::Quit)
			return;
		else if (msg.type != RPiCamApp::MsgType::RequestComplete)
			throw std::runtime_error("unrecognised message!");
		
		int key = get_key_or_signal(options_ptr, p); // Keypress monitoring

		LOG(2, "Frame " << count);
		auto now = std::chrono::high_resolution_clock::now();
		
		// Logic to stop based on time or frame count
		bool timeout = !options.frames && options.timeout &&
					   ((now - start_time) > options.timeout.value);
		bool frameout = options.frames && count >= options.frames;
		
		if (timeout || frameout || key == 'x' || key == 'X')
		{
			if (timeout)
				LOG(1, "Halting: reached timeout.");
			app.StopCamera();
			app.StopEncoder();
			return;
		}

		// --- WRITE DNG LOGIC ---
		CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);
		
		// The Raw stream is where the unencoded data lives in this configuration.
		Stream *raw_stream = app.RawStream();
		
		if (completed_request->buffers.count(raw_stream))
		{
			// 1. Get the Raw Buffer Data
			BufferReadSync r(&app, completed_request->buffers[raw_stream]);
			const std::vector<libcamera::Span<uint8_t>> mem = r.Get();
			
			// 2. Get Stream Configuration
			StreamConfiguration const &cfg = raw_stream->configuration();
			
			// 3. Populate StreamInfo (using fixed struct member access)
			StreamInfo info;
			info.width = cfg.size.width;
			info.height = cfg.size.height;
			info.stride = cfg.stride;
			info.pixel_format = cfg.pixelFormat;
			
			// 4. Generate filename
			std::stringstream filename_ss;
			std::string base = options.output.empty() ? "frame" : options.output;
			size_t lastindex = base.find_last_of("."); 
			if (lastindex != std::string::npos) 
				base = base.substr(0, lastindex); 

			filename_ss << base << "_" << std::setw(4) << std::setfill('0') << count << ".dng";
			std::string filename = filename_ss.str();

			// 5. Save DNG File
			try {
				dng_save(
					mem, 
					info, 
					completed_request->metadata, 
					filename, 
					app.CameraModel(), 
					// FIX: Use reinterpret_cast to force the type conversion 
					// needed by the dng_save function when using VideoOptions.
					reinterpret_cast<StillOptions const *>(options_ptr) 
				);
			} catch (std::exception const &e) {
				LOG_ERROR("Failed to write DNG: " << e.what());
			}
		}

		// Show preview on HDMI/Screen
		// Use the video stream (Stream 0) for preview, which is lower resolution by default
		app.ShowPreview(completed_request, app.VideoStream());
	}
}

// --- Main function ---

int main(int argc, char *argv[])
{
	try
	{
		RPiCamCinema app;
		VideoOptions *options = app.GetOptions();
		if (options->Parse(argc, argv))
		{
            // Apply standard raw video options defaults
			options->Set().codec = "yuv420";
			options->Set().denoise = "cdn_off";
			
            // We want a preview for rpicam-cinema, so DON'T force nopreview=true
            
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
