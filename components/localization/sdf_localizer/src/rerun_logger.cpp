#include "rerun_logger.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <QDebug>

// ──────────────────────────────────────────────────────────────────────────────
// Minimal JSON helpers (no external library required)
// ──────────────────────────────────────────────────────────────────────────────
namespace {

static std::string jf(float v, int prec = 5) {
    if (std::isnan(v) || std::isinf(v)) return "null";
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(prec) << v;
    return ss.str();
}
static std::string ji(int64_t v)  { return std::to_string(v); }
static std::string jb(bool v)     { return v ? "true" : "false"; }

// Encode float array as base64 for embedding in JSON
static const char B64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
static std::string base64(const float* data, size_t n)
{
    const uint8_t* src = reinterpret_cast<const uint8_t*>(data);
    size_t len = n * sizeof(float);
    std::string out;
    out.reserve(((len + 2) / 3) * 4 + 2);
    out += '"';
    for (size_t i = 0; i < len; i += 3) {
        uint32_t v = src[i] << 16;
        if (i+1 < len) v |= src[i+1] << 8;
        if (i+2 < len) v |= src[i+2];
        out += B64[(v >> 18) & 0x3F];
        out += B64[(v >> 12) & 0x3F];
        out += (i+1 < len) ? B64[(v >> 6) & 0x3F] : '=';
        out += (i+2 < len) ? B64[v & 0x3F] : '=';
    }
    out += '"';
    return out;
}

static std::string serialize_frame(const RerunFrame& f)
{
    std::ostringstream j;
    j << '{';
    j << "\"ts_ms\":"      << ji(f.ts_ms)       << ',';
    j << "\"frame_id\":"   << ji(f.frame_id)     << ',';
    j << "\"x\":"          << jf(f.x)            << ',';
    j << "\"y\":"          << jf(f.y)            << ',';
    j << "\"theta\":"      << jf(f.theta)        << ',';
    j << "\"pred_x\":"     << jf(f.pred_x)       << ',';
    j << "\"pred_y\":"     << jf(f.pred_y)       << ',';
    j << "\"pred_theta\":" << jf(f.pred_theta)   << ',';
    j << "\"early_exit\":" << jb(f.early_exit)   << ',';
    j << "\"ws\":"         << ji(f.window_size)  << ',';
    j << "\"iters\":"      << ji(f.iters)        << ',';
    j << "\"loss_init\":"  << jf(f.loss_init)    << ',';
    j << "\"final_loss\":" << jf(f.final_loss)   << ',';
    j << "\"l_bnd\":"      << jf(f.loss_boundary)<< ',';
    j << "\"l_obs\":"      << jf(f.loss_obs)     << ',';
    j << "\"l_mot\":"      << jf(f.loss_motion)  << ',';
    j << "\"l_cor\":"      << jf(f.loss_corner)  << ',';
    j << "\"sdf_mse\":"    << jf(f.sdf_mse)      << ',';
    j << "\"innov_x\":"    << jf(f.innov_x)      << ',';
    j << "\"innov_y\":"    << jf(f.innov_y)      << ',';
    j << "\"innov_theta\":"<< jf(f.innov_theta)  << ',';
    j << "\"innov_norm\":" << jf(f.innov_norm)   << ',';
    j << "\"cov_xx\":"     << jf(f.cov_xx)       << ',';
    j << "\"cov_xy\":"     << jf(f.cov_xy)       << ',';
    j << "\"cov_yy\":"     << jf(f.cov_yy)       << ',';
    j << "\"cov_tt\":"     << jf(f.cov_tt)       << ',';
    j << "\"cond_num\":"   << jf(f.cond_num, 2)  << ',';
    j << "\"t_update\":"   << jf(f.t_update_ms, 2) << ',';
    j << "\"t_adam\":"     << jf(f.t_adam_ms,   2) << ',';
    j << "\"t_cov\":"      << jf(f.t_cov_ms,    2) << ',';
    j << "\"t_bkd\":"      << jf(f.t_breakdown_ms,2) << ',';

    // Point cloud — [N, 3] as base64 float32 LE
    j << "\"n_pts\":" << ji(f.lidar_points.size()) << ',';
    if (!f.lidar_points.empty()) {
        j << "\"pts\":" << base64(f.lidar_points[0].data(),
                                   f.lidar_points.size() * 3) << ',';
    } else {
        j << "\"pts\":null,";
    }

    // SDF grid (optional)
    if (f.has_sdf_grid && !f.sdf_values.empty()) {
        j << "\"sdf_w\":"          << ji(f.sdf_w)           << ',';
        j << "\"sdf_h\":"          << ji(f.sdf_h)           << ',';
        j << "\"sdf_ox\":"         << jf(f.sdf_origin_x)    << ',';
        j << "\"sdf_oy\":"         << jf(f.sdf_origin_y)    << ',';
        j << "\"sdf_cell\":"       << jf(f.sdf_cell_size)   << ',';
        j << "\"sdf\":" << base64(f.sdf_values.data(), f.sdf_values.size()) << ',';
    } else {
        j << "\"sdf\":null,";
    }

    // Room polygon (optional)
    j << "\"room_poly_n\":" << ji(static_cast<int64_t>(f.room_polygon.size())) << ',';
    if (f.has_room_polygon && !f.room_polygon.empty())
    {
        j << "\"room_poly\":" << base64(f.room_polygon[0].data(), f.room_polygon.size() * 2);
    }
    else
    {
        j << "\"room_poly\":null";
    }

    j << '}';
    return j.str();
}

} // anonymous namespace

// ──────────────────────────────────────────────────────────────────────────────
// RerunLogger implementation
// ──────────────────────────────────────────────────────────────────────────────

void RerunLogger::init(const Config& cfg)
{
    // Safe re-init: ensure old sender is stopped before creating a new thread.
    stop();

    cfg_ = cfg;
    if (!cfg_.enabled) return;

    stop_requested_ = false;
    frame_counter_ = 0;
    sender_thread_ = std::thread(&RerunLogger::sender_loop, this);
    qInfo() << "[RerunLogger] started → " << cfg_.host.c_str() << ":" << cfg_.port;
}

void RerunLogger::stop()
{
    stop_requested_ = true;
    queue_cv_.notify_all();
    if (sender_thread_.joinable()) sender_thread_.join();
    if (sock_fd_ >= 0) { ::close(sock_fd_); sock_fd_ = -1; }
    connected_ = false;
}

void RerunLogger::log_frame(RerunFrame frame)
{
    if (!cfg_.enabled) return;
    frame.frame_id = ++frame_counter_;

    std::lock_guard<std::mutex> lk(queue_mtx_);
    if (static_cast<int>(queue_.size()) >= cfg_.max_queue)
        queue_.pop_front();   // drop oldest if bridge is slow
    queue_.push_back(std::move(frame));
    queue_cv_.notify_one();
}

void RerunLogger::sender_loop()
{
    while (!stop_requested_) {
        // Wait for a frame
        std::unique_lock<std::mutex> lk(queue_mtx_);
        queue_cv_.wait_for(lk, std::chrono::milliseconds(500),
                           [this]{ return !queue_.empty() || stop_requested_.load(); });
        if (stop_requested_ && queue_.empty()) break;
        if (queue_.empty()) continue;

        RerunFrame frame = std::move(queue_.front());
        queue_.pop_front();
        lk.unlock();

        // Ensure connection
        if (!connected_) {
            if (!try_connect()) {
                std::this_thread::sleep_for(std::chrono::seconds(2));
                continue;
            }
        }

        if (!send_frame(frame)) {
            qWarning() << "[RerunLogger] send_frame failed:" << std::strerror(errno)
                       << "— reconnecting";
            ::close(sock_fd_);
            sock_fd_ = -1;
            connected_ = false;
        }
    }
}

bool RerunLogger::try_connect()
{
    if (sock_fd_ >= 0) { ::close(sock_fd_); sock_fd_ = -1; }

    sock_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock_fd_ < 0) return false;

    int flag = 1;
    ::setsockopt(sock_fd_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(static_cast<uint16_t>(cfg_.port));
    ::inet_pton(AF_INET, cfg_.host.c_str(), &addr.sin_addr);

    if (::connect(sock_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        ::close(sock_fd_);
        sock_fd_ = -1;
        return false;
    }

    connected_ = true;
    qInfo() << "[RerunLogger] connected to bridge at"
            << cfg_.host.c_str() << ":" << cfg_.port;
    return true;
}

bool RerunLogger::send_frame(const RerunFrame& f)
{
    const std::string json = serialize_frame(f);
    const uint32_t sz = static_cast<uint32_t>(json.size());
    // Little-endian 4-byte length header
    uint8_t hdr[4] = {
        static_cast<uint8_t>(sz & 0xFF),
        static_cast<uint8_t>((sz >> 8)  & 0xFF),
        static_cast<uint8_t>((sz >> 16) & 0xFF),
        static_cast<uint8_t>((sz >> 24) & 0xFF)
    };
    return tcp_write(hdr, 4) && tcp_write(json.data(), sz);
}

bool RerunLogger::tcp_write(const void* data, size_t len)
{
    const char* ptr = static_cast<const char*>(data);
    size_t remaining = len;
    while (remaining > 0) {
        ssize_t n = ::send(sock_fd_, ptr, remaining, MSG_NOSIGNAL);
        if (n <= 0) {
            if (errno == EINTR) continue;
            return false;
        }
        ptr       += n;
        remaining -= n;
    }
    return true;
}
