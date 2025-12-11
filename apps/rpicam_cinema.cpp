/* SPDX-License-Identifier: BSD-2-Clause */
/*
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

// 1. Forward Declaration
class RPiCamCinemaApp;

// ----------------------------------------------------------------------------
// Threaded Saver Class Definition
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

    ThreadedSaver(StillOptions *opts);
    ~ThreadedSaver();

    void enqueue(SaveTask task);

private:
    void process();

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

// ----------------------------------------------------------------------------
// Threaded Saver Implementation
// ----------------------------------------------------------------------------

ThreadedSaver::ThreadedSaver(StillOptions *opts) : options_(opts), running_(true) {
    worker_ = std::thread(&ThreadedSaver::process, this);
}

ThreadedSaver::~ThreadedSaver() {
    {
        std::unique_lock<std::mutex> lock(mutex_);
        running_ = false;
    }
    cv_.notify_one();
    if (worker_.joinable()) worker_.join();
}

void ThreadedSaver::enqueue(SaveTask task) {
    std::unique_lock<std::mutex> lock(mutex_);
    // Limit queue size to prevent OOM
    if (queue_.size() < 40) { 
        queue_.push(std::move(task));
        cv_.notify_one();
    } else {
        LOG(2, "Disk I/O too slow, dropping frame!");
    }
}

void ThreadedSaver::process() {
    while (true) {
        SaveTask task;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this] { return !queue_.empty() || !running_; });
            if (!running_ && queue_.empty()) break;
            task = std::move(queue_.front());
            queue_.pop();
        }

        StreamInfo info = task.app->GetStreamInfo(task.stream);
        BufferReadSync r(task.app, task.payload->buffers[task.stream]);
        const std::vector<libcamera::Span<uint8_t>> mem = r.Get();
        
        dng_save(mem, info, task.payload->metadata, task.filename, task.app->CameraModel(), options_);
        LOG(2, "Saved " << task.filename << " (Queue: " << queue_.size() << ")");
    }
}

// ----------------------------------------------------------------------------
// Helper Functions
// ----------------------------------------------------------------------------

static std::string generate_filename(StillOptions const *options)
{
    const std::string encoding = "dng";
    char filename[128];
    std::string folder = options->Get().output; 
    if (!folder.empty() && folder.back() != '/')
        folder += "/";

    snprintf(filename, sizeof(filename), "%s%04d.%s", folder.c_str(), options->Get().framestart, encoding.c_str());
    return std::string(filename);
}

static int signal_received;
static void default_signal_handler(int signal_number)
{
    signal_received = signal_number;
}

// ----------------------------------------------------------------------------
// Event Loop
// ----------------------------------------------------------------------------

static void event_loop(RPiCamCinemaApp &app)
{
    StillOptions *options = app.GetOptions();

    unsigned int flags = RPiCamApp::FLAG_STILL_RAW;
    
    app.OpenCamera();
    app.ConfigureStill(flags);
    app.StartCamera();

    ThreadedSaver saver(options);

    auto start_time = std::chrono::high_resolution_clock::now();
    auto last_frame_time = start_time;
    
    // FIX 1 (Line 174): .get() returns an integer, so we don't use .count()
    // It is already the millisecond count.
    uint64_t frame_interval_ms = options->Get().timelapse.get();

    LOG(1, "Starting High Speed Cinema Capture. Press Ctrl+C or 'x' + Enter to stop.");

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

        // Handle Exit
        poll(p, 1, 0);
        if (p[0].revents & POLLIN)
        {
            char *user_string = nullptr;
            size_t len;
            [[maybe_unused]] size_t r = getline(&user_string, &len, stdin);
            if (user_string[0] == 'x' || user_string[0] == 'X') return;
        }
        if (signal_received) return;

        if (app.RawStream())
        {
            // Time check for frame interval
            double elapsed = std::chrono::duration<double, std::milli>(now - last_frame_time).count();
            
            if (elapsed >= frame_interval_ms)
            {
                last_frame_time = now;
                std::string filename = generate_filename(options);
                
                ThreadedSaver::SaveTask task;
                task.payload = completed_request; 
                task.filename = filename;
                task.app = &app;
                task.stream = app.RawStream(); 
                
                saver.enqueue(std::move(task));

                options->Set().framestart++;
                
                // FIX 2 & 3: Handle Type Mismatch for Timeout
                // Compare the raw integer (.get()) to 0
                if (options->Get().timeout.get() > 0)
                {
                    // Convert the raw integer timeout to a chrono duration for comparison with 'now'
                    // This resolves the error on line 231 (now - start_time) > timeout_integer
                    auto timeout_duration = std::chrono::milliseconds(options->Get().timeout.get());
                    if ((now - start_time) > timeout_duration)
                        return;
                }
            }
        }
    }
}

// ----------------------------------------------------------------------------
// Main Function
// ----------------------------------------------------------------------------

int main(int argc, char *argv[])
{
    try
    {
        RPiCamCinemaApp app;
        StillOptions *options = app.GetOptions();
        if (options->Parse(argc, argv))
        {
            if (options->Get().verbose >= 2) options->Get().Print();
            
            // FIX 4 (Line 248): Must use Set() for mutable access. 
            // options->Get() returns a const reference, preventing the call to .set().
            if (!options->Get().timeout) options->Set().timeout.set(0);

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
