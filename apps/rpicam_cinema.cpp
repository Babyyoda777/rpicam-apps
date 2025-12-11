/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * rpicam_Cinema.cpp - libcamera Cinema high-speed DNG capture.
 */
#include <chrono>
#include <filesystem>
#include <poll.h>
#include <signal.h>
#include <sys/signalfd.h>
#include <sys/stat.h>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include "core/frame_info.hpp"
#include "core/rpicam_app.hpp"
#include "core/still_options.hpp"

#include "output/output.hpp"

#include "image/image.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using libcamera::Stream;

namespace fs = std::filesystem;

// ----------------------------------------------------------------------------
// Threaded Saver Class
// Handles disk I/O in the background so the camera doesn't stall.
// ----------------------------------------------------------------------------
class ThreadedSaver
{
public:
    struct SaveTask {
        CompletedRequestPtr payload;
        std::string filename;
        RPiCamCinemaApp* app;
        Stream* stream;
    };

    ThreadedSaver(StillOptions *opts) : options_(opts), running_(true) {
        worker_ = std::thread(&ThreadedSaver::process, this);
    }

    ~ThreadedSaver() {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            running_ = false;
        }
        cv_.notify_one();
        if (worker_.joinable()) worker_.join();
    }

    void enqueue(SaveTask task) {
        std::unique_lock<std::mutex> lock(mutex_);
        // If queue is too large (disk too slow), we drop frames to keep camera alive
        if (queue_.size() < 20) { 
            queue_.push(std::move(task));
            cv_.notify_one();
        } else {
            LOG(2, "Disk I/O too slow, dropping frame!");
        }
    }

private:
    void process() {
        while (true) {
            SaveTask task;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [this] { return !queue_.empty() || !running_; });
                if (!running_ && queue_.empty()) break;
                task = std::move(queue_.front());
                queue_.pop();
            }

            // Perform the heavy DNG save
            StreamInfo info = task.app->GetStreamInfo(task.stream);
            BufferReadSync r(task.app, task.payload->buffers[task.stream]);
            const std::vector<libcamera::Span<uint8_t>> mem = r.Get();
            
            dng_save(mem, info, task.payload->metadata, task.filename, task.app->CameraModel(), options_);
            LOG(2, "Saved " << task.filename << " (Queue: " << queue_.size() << ")");
        }
    }

    StillOptions *options_;
    std::thread worker_;
    std::queue<SaveTask> queue_;
    std::mutex mutex_;
    std::condition_variable cv_;
    bool running_;
};

// ----------------------------------------------------------------------------
// Main Application Class
// ----------------------------------------------------------------------------

class RPiCamCinemaApp : public RPiCamApp
{
public:
    RPiCamCinemaApp() : RPiCamApp(std::make_unique<StillOptions>())
    {
        static_cast<StillOptions *>(RPiCamApp::GetOptions())->Set().encoding = "dng";
    }

    StillOptions *GetOptions() const { return static_cast<StillOptions *>(RPiCamApp::GetOptions()); }
};

static std::string generate_filename(StillOptions const *options)
{
    const std::string encoding = "dng";
    char filename[128];
    std::string folder = options->Get().output; 
    if (!folder.empty() && folder.back() != '/')
        folder += "/";

    // For high speed capture, we strictly use the frame counter
    snprintf(filename, sizeof(filename), "%s%04d.%s", folder.c_str(), options->Get().framestart, encoding.c_str());

    filename[sizeof(filename) - 1] = 0;
    return std::string(filename);
}

// ----------------------------------------------------------------------------
// Event Loop
// ----------------------------------------------------------------------------

static int signal_received;
static void default_signal_handler(int signal_number)
{
    signal_received = signal_number;
}

static void event_loop(RPiCamCinemaApp &app)
{
    StillOptions *options = app.GetOptions();

    // 1. Force RAW/Cinema configuration immediately.
    // We ignore Viewfinder entirely to ensure the pipeline is optimized for the capture stream.
    unsigned int Cinema_flags = RPiCamApp::FLAG_Cinema_RAW; 
    
    app.OpenCamera();
    app.ConfigureCinema(Cinema_flags);
    app.StartCamera();

    ThreadedSaver saver(options);

    auto start_time = std::chrono::high_resolution_clock::now();
    auto last_frame_time = start_time;
    
    // Calculate minimum frame duration based on requested FPS (if timelapse is used as FPS control)
    // If --timelapse 0 is used, it goes as fast as the camera/exposure allows.
    // If --timelapse 40 is used, it targets 25fps.
    uint64_t frame_interval_ms = options->Get().timelapse; 

    LOG(1, "Starting High Speed Cinema Capture. Press Ctrl+C or 'x' + Enter to stop.");

    // Monitoring for keypresses and signals.
    signal(SIGUSR1, default_signal_handler);
    pollfd p[1] = { { STDIN_FILENO, POLLIN, 0 } };

    for (unsigned int count = 0;; count++)
    {
        RPiCamApp::Msg msg = app.Wait();

        if (msg.type == RPiCamApp::MsgType::Timeout)
        {
            LOG_ERROR("ERROR: Device timeout detected, attempting restart!");
            app.StopCamera();
            app.StartCamera();
            continue;
        }
        if (msg.type == RPiCamApp::MsgType::Quit)
            return;

        CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);
        auto now = std::chrono::high_resolution_clock::now();

        // Handle Keyboard Exit
        poll(p, 1, 0);
        if (p[0].revents & POLLIN)
        {
            char *user_string = nullptr;
            size_t len;
            [[maybe_unused]] size_t r = getline(&user_string, &len, stdin);
            if (user_string[0] == 'x' || user_string[0] == 'X') return;
        }
        if (signal_received) return;

        // 2. Logic to determine if we save this frame
        // In high speed mode, we stream "Cinema" buffers.
        if (app.CinemaStream())
        {
            // Check timelapse interval (software rate limiting)
            // Ideally, set --framerate on the command line to control this at the sensor level
            double elapsed = std::chrono::duration<double, std::milli>(now - last_frame_time).count();
            
            if (elapsed >= frame_interval_ms)
            {
                last_frame_time = now;
                std::string filename = generate_filename(options);
                
                // 3. Offload to background thread
                ThreadedSaver::SaveTask task;
                task.payload = completed_request; // Shared ptr keeps buffer alive
                task.filename = filename;
                task.app = &app;
                task.stream = app.CinemaStream(); // Use the raw stream
                
                saver.enqueue(std::move(task));

                // Increment counter
                options->Set().framestart++;
                
                // Stop if we hit a frame limit (optional, based on user needs)
                if (options->Get().timeout && (now - start_time) > std::chrono::milliseconds(options->Get().timeout.value))
                    return;
            }
        }
    }
}

int main(int argc, char *argv[])
{
    try
    {
        RPiCamCinemaApp app;
        StillOptions *options = app.GetOptions();
        if (options->Parse(argc, argv))
        {
            // Force disable ZSL as we are doing continuous direct capture
            // Force immediate mode logic
            if (options->Get().verbose >= 2) options->Get().Print();
            
            // Remove any "timeout" default that stops the app immediately
            // If user didn't specify timeout, run forever (0)
            if (!options->Get().timeout) options->Get().timeout = 0;

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
