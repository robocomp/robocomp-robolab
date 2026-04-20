#pragma once
/**
 * RerunLogger — sends per-frame telemetry to a Python bridge that logs to Rerun.
 *
 * Protocol: TCP socket, localhost.  Each message is:
 *   [4 bytes LE]  payload_size
 *   [payload]     JSON UTF-8 string
 *
 * The JSON schema is described in scripts/rerun_bridge.py.
 * Call log_frame() from the localization thread; it is non-blocking (data is
 * queued and sent by a background sender thread).
 */

#include <string>
#include <vector>
#include <array>
#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <cstdint>

// ──────────────────────────────────────────────────────────────────────────────
/// All data the localizer wants to export for one update() cycle.
struct RerunFrame
{
    // ---- identity ----
    int64_t  ts_ms        = 0;
    int      frame_id     = 0;

    // ---- pose ----
    float    x = 0, y = 0, theta = 0;
    float    pred_x = 0, pred_y = 0, pred_theta = 0;

    // ---- pipeline flags ----
    bool     early_exit   = false;
    int      window_size  = 0;
    int      iters        = 0;

    // ---- loss ----
    float    loss_init    = 0;
    float    final_loss   = 0;
    float    loss_boundary= 0;
    float    loss_obs     = 0;
    float    loss_motion  = 0;
    float    loss_corner  = 0;

    // ---- localization quality ----
    float    sdf_mse      = 0;
    float    innov_x      = 0;
    float    innov_y      = 0;
    float    innov_theta  = 0;
    float    innov_norm   = 0;
    float    cov_xx       = 0;
    float    cov_xy       = 0;
    float    cov_yy       = 0;
    float    cov_tt       = 0;
    float    cond_num     = 0;

    // ---- timing (ms) ----
    float    t_update_ms  = 0;
    float    t_adam_ms    = 0;
    float    t_cov_ms     = 0;
    float    t_breakdown_ms = 0;

    // ---- lidar points in ROBOT frame [N×3: x,y,z] ----
    std::vector<std::array<float,3>> lidar_points;

    // ---- SDF grid (optional, large — sent every N frames) ----
    bool     has_sdf_grid = false;
    int      sdf_w = 0, sdf_h = 0;
    float    sdf_origin_x = 0, sdf_origin_y = 0;   // world coords of grid (0,0)
    float    sdf_cell_size = 0;                      // metres per cell
    std::vector<float> sdf_values;                   // row-major, size = sdf_w × sdf_h

    // ---- Room polygon (optional, sent once / on updates) ----
    bool     has_room_polygon = false;
    std::vector<std::array<float, 2>> room_polygon;  // world-frame corners [x,y]
};

// ──────────────────────────────────────────────────────────────────────────────
class RerunLogger
{
public:
    struct Config {
        bool        enabled        = true;
        std::string host           = "127.0.0.1";
        int         port           = 9877;          // rerun_bridge.py listens here
        int         sdf_every_n    = 20;            // send SDF grid every N frames
        int         sdf_resolution = 150;           // grid cells per axis
        int         max_queue      = 30;            // drop oldest if queue full
    };

    RerunLogger() = default;
    ~RerunLogger() { stop(); }

    // Call once from the main thread after the room model is ready.
    void init(const Config& cfg);

    // Queue a frame for sending (called from localization thread, non-blocking).
    void log_frame(RerunFrame frame);

    // Graceful shutdown.
    void stop();

    bool is_enabled() const { return cfg_.enabled && connected_.load(); }

private:
    Config          cfg_;
    int             sock_fd_   = -1;
    std::atomic<bool> connected_{false};
    std::atomic<bool> stop_requested_{false};

    std::deque<RerunFrame>   queue_;
    std::mutex               queue_mtx_;
    std::condition_variable  queue_cv_;
    std::thread              sender_thread_;
    std::atomic<int>          frame_counter_{0};

    // Background thread: dequeue frames and send over TCP.
    void sender_loop();

    // Serialize one frame to JSON and send.
    bool send_frame(const RerunFrame& f);

    // Low-level TCP write (retries on EINTR).
    bool tcp_write(const void* data, size_t len);

    // Try to (re)connect to the bridge.
    bool try_connect();
};
