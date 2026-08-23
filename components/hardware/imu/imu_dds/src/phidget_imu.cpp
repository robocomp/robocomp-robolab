/*
 *    Copyright (C) 2026 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "phidget_imu.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <format>
#include <limits>
#include <mutex>
#include <print>
#include <string>

#include <phidget22.h>

namespace
{
// The phidget22 driver reports acceleration in g and angular rate in deg/s. The media
// plane is SI throughout (imu_sample.h), so convert here — on the way IN, once.
constexpr double G_TO_MPS2  = 9.80665;
constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;

// A field the device cannot currently measure comes back as PUNK_DBL (1e300), NOT as an
// error. The magnetometer does this routinely: it saturates near motors and while the
// compass is being calibrated. Passing 1e300 through as a float would become inf and
// poison any consumer that averages it, so it is mapped to NaN — which is what
// imu_sample.h documents for "unavailable" and what a consumer can actually test for.
inline float si(double v, double scale)
{
    return (v >= PUNK_DBL) ? std::numeric_limits<float>::quiet_NaN()
                           : static_cast<float>(v * scale);
}

std::uint64_t wall_now_ms()
{
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
}

std::string phidget_error(PhidgetReturnCode rc)
{
    const char* desc = nullptr;
    if (Phidget_getErrorDescription(rc, &desc) == EPHIDGET_OK and desc != nullptr)
        return desc;
    return "code " + std::to_string(static_cast<int>(rc));
}
}  // namespace

struct PhidgetImu::Impl
{
    Config cfg;
    PhidgetSpatialHandle spatial = nullptr;

    mutable std::mutex   mtx;
    ImuSample            latest;             // guarded by mtx
    std::uint64_t        seq = 0;            // guarded by mtx; bumped per callback
    std::uint64_t        last_read_seq = 0;  // compute()-thread only
    std::string          label;              // guarded by mtx

    std::atomic<bool>    attached{false};

    // Anchor mapping the device clock onto the wall clock. The spatial callback's
    // `timestamp` counts milliseconds since the channel attached — NOT an epoch — but
    // ImuFrame.stamp_ms is contractually WALL epoch ms, and consumers use it for
    // staleness and cross-stream alignment. So capture (wall, device) once at the first
    // sample and offset every later one by that difference: absolute time becomes
    // correct while the INTER-SAMPLE deltas stay the device's own, which is what anything
    // integrating a rate actually depends on. Re-anchored on every attach, because the
    // device clock restarts there.
    bool          anchored = false;
    double        hw_t0_ms = 0.0;
    std::uint64_t wall_t0_ms = 0;

    void on_spatial(const double acc[3], const double gyro[3], const double mag[3], double hw_ts_ms);
    void on_algorithm(const double quat[4]);

    // The phidget22 driver takes plain C function pointers. They are static MEMBERS rather
    // than free functions so they can reach this (private) type; `ctx` carries the instance.
    static void CCONV cb_spatial(PhidgetSpatialHandle, void* ctx,
                                 const double acceleration[3], const double angularRate[3],
                                 const double magneticField[3], double timestamp);
    static void CCONV cb_algorithm(PhidgetSpatialHandle, void* ctx, const double quaternion[4], double);
    static void CCONV cb_attach(PhidgetHandle ch, void* ctx);
    static void CCONV cb_detach(PhidgetHandle, void* ctx);
    static void CCONV cb_error(PhidgetHandle, void*, Phidget_ErrorEventCode code, const char* description);
};

// ── driver callbacks (called on the phidget22 thread) ────────────────────────
void CCONV PhidgetImu::Impl::cb_spatial(PhidgetSpatialHandle, void* ctx,
                                        const double acceleration[3], const double angularRate[3],
                                        const double magneticField[3], double timestamp)
{
    static_cast<PhidgetImu::Impl*>(ctx)->on_spatial(acceleration, angularRate, magneticField, timestamp);
}

void CCONV PhidgetImu::Impl::cb_algorithm(PhidgetSpatialHandle, void* ctx, const double quaternion[4], double)
{
    static_cast<PhidgetImu::Impl*>(ctx)->on_algorithm(quaternion);
}

void CCONV PhidgetImu::Impl::cb_attach(PhidgetHandle ch, void* ctx)
{
    auto* impl = static_cast<PhidgetImu::Impl*>(ctx);

    // DataInterval can only be set once attached, and the device's own floor wins: asking
    // for less than MinDataInterval is an error, not a clamp, so ask for the max of the two.
    std::uint32_t min_di = 0;
    if (PhidgetSpatial_getMinDataInterval(reinterpret_cast<PhidgetSpatialHandle>(ch), &min_di) != EPHIDGET_OK)
        min_di = 1;
    const std::uint32_t want = std::max<std::uint32_t>(static_cast<std::uint32_t>(impl->cfg.data_interval_ms), min_di);
    if (const auto rc = PhidgetSpatial_setDataInterval(reinterpret_cast<PhidgetSpatialHandle>(ch), want);
        rc != EPHIDGET_OK)
        std::print(stderr, "[phidget] could not set DataInterval={} ms: {}\n", want, phidget_error(rc));

    std::int32_t serial = -1; int hub = -1;
    Phidget_getDeviceSerialNumber(ch, &serial);
    Phidget_getHubPort(ch, &hub);

    {
        std::scoped_lock lk(impl->mtx);
        impl->label = std::format("Spatial serial {} hub port {}", serial, hub);
        impl->anchored = false;   // device clock restarts on attach -> re-anchor
    }
    impl->attached.store(true, std::memory_order_relaxed);
    std::print("[phidget] attached: {} DataInterval={} ms ({:.0f} Hz)\n",
               impl->label, want, 1000.0 / static_cast<double>(want));
}

void CCONV PhidgetImu::Impl::cb_detach(PhidgetHandle, void* ctx)
{
    auto* impl = static_cast<PhidgetImu::Impl*>(ctx);
    impl->attached.store(false, std::memory_order_relaxed);
    std::print(stderr, "[phidget] detached — the driver will reconnect on its own\n");
}

void CCONV PhidgetImu::Impl::cb_error(PhidgetHandle, void*, Phidget_ErrorEventCode code, const char* description)
{
    std::print(stderr, "[phidget] error [{}]: {}\n", static_cast<int>(code),
               description != nullptr ? description : "(no description)");
}

void PhidgetImu::Impl::on_spatial(const double acc[3], const double gyro[3],
                                  const double mag[3], double hw_ts_ms)
{
    const std::uint64_t wall = wall_now_ms();
    std::scoped_lock lk(mtx);

    if (not anchored)
    {
        hw_t0_ms   = hw_ts_ms;
        wall_t0_ms = wall;
        anchored   = true;
    }
    const double since_t0 = hw_ts_ms - hw_t0_ms;
    latest.stamp_ms = wall_t0_ms + static_cast<std::uint64_t>(since_t0 < 0.0 ? 0.0 : since_t0);

    // A real sensor has no simulation clock; 0 IS the "not simulated" flag consumers key on.
    latest.sim_stamp_ms = 0;

    for (int i = 0; i < 3; ++i)
    {
        latest.acc[i]  = si(acc[i],  G_TO_MPS2);
        latest.gyro[i] = si(gyro[i], DEG_TO_RAD);
        latest.mag[i]  = si(mag[i],  1.0);          // Gauss, already the media-plane unit
    }
    // Spatial carries no temperature channel in phidget22 (temperature is a separate
    // PhidgetTemperatureSensor device), so leave it at 0 rather than invent a reading.
    latest.temperature = 0.f;
    latest.gyro_var    = cfg.gyro_var;
    latest.acc_var     = cfg.acc_var;
    // rpy is written by on_algorithm() when the AHRS is on, and left at 0 otherwise. It is
    // deliberately NOT touched here: overwriting it every spatial sample would erase the
    // orientation between the two interleaved callbacks.
    ++seq;
}

void PhidgetImu::Impl::on_algorithm(const double q[4])
{
    // quaternion is [x, y, z, w]. Standard ZYX (yaw-pitch-roll) extraction.
    const double x = q[0], y = q[1], z = q[2], w = q[3];

    const double sinr_cosp = 2.0 * (w * x + y * z);
    const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    const double roll = std::atan2(sinr_cosp, cosr_cosp);

    // asin would produce NaN if rounding pushes |sinp| past 1, so clamp to the pole instead.
    const double sinp = 2.0 * (w * y - z * x);
    const double pitch = (std::abs(sinp) >= 1.0) ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);

    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    const double yaw = std::atan2(siny_cosp, cosy_cosp);

    std::scoped_lock lk(mtx);
    latest.rpy[0] = static_cast<float>(roll);
    latest.rpy[1] = static_cast<float>(pitch);
    latest.rpy[2] = static_cast<float>(yaw);
}

// ── public API ───────────────────────────────────────────────────────────────
PhidgetImu::PhidgetImu() : pimpl_(std::make_unique<Impl>()) {}
PhidgetImu::~PhidgetImu() { stop(); }

bool PhidgetImu::start(const Config& cfg)
{
    stop();               // idempotent: allow a retry after a failed open
    pimpl_->cfg = cfg;

    if (const auto rc = PhidgetSpatial_create(&pimpl_->spatial); rc != EPHIDGET_OK)
    {
        std::print(stderr, "[phidget] PhidgetSpatial_create failed: {}\n", phidget_error(rc));
        pimpl_->spatial = nullptr;
        return false;
    }
    auto h = reinterpret_cast<PhidgetHandle>(pimpl_->spatial);

    if (cfg.serial >= 0)   Phidget_setDeviceSerialNumber(h, cfg.serial);
    if (cfg.hub_port >= 0) Phidget_setHubPort(h, cfg.hub_port);

    Phidget_setOnAttachHandler(h, Impl::cb_attach, pimpl_.get());
    Phidget_setOnDetachHandler(h, Impl::cb_detach, pimpl_.get());
    Phidget_setOnErrorHandler(h, Impl::cb_error, pimpl_.get());
    PhidgetSpatial_setOnSpatialDataHandler(pimpl_->spatial, Impl::cb_spatial, pimpl_.get());
    if (cfg.use_ahrs)
        PhidgetSpatial_setOnAlgorithmDataHandler(pimpl_->spatial, Impl::cb_algorithm, pimpl_.get());

    if (const auto rc = Phidget_openWaitForAttachment(h, static_cast<std::uint32_t>(cfg.open_timeout_ms));
        rc != EPHIDGET_OK)
    {
        std::print(stderr, "[phidget] no Spatial attached within {} ms: {}\n",
                   cfg.open_timeout_ms, phidget_error(rc));
        stop();
        return false;
    }

    // The algorithm must be selected AFTER attach (it is a device property, and the handle
    // has no device to talk to before then). AHRS fuses the magnetometer for an absolute
    // heading; IMU mode is gyro+accel only. Failure here is not fatal — the raw channels
    // keep flowing, only rpy stays 0 — so it warns rather than aborting the open.
    if (cfg.use_ahrs)
        if (const auto rc = PhidgetSpatial_setAlgorithm(pimpl_->spatial, SPATIAL_ALGORITHM_AHRS);
            rc != EPHIDGET_OK)
            std::print(stderr, "[phidget] AHRS unavailable ({}): orientation (rpy) stays 0\n",
                       phidget_error(rc));

    return true;
}

void PhidgetImu::stop()
{
    if (pimpl_->spatial == nullptr)
        return;
    Phidget_close(reinterpret_cast<PhidgetHandle>(pimpl_->spatial));
    PhidgetSpatial_delete(&pimpl_->spatial);
    pimpl_->spatial = nullptr;
    pimpl_->attached.store(false, std::memory_order_relaxed);
    {
        std::scoped_lock lk(pimpl_->mtx);
        pimpl_->anchored = false;
    }
}

bool PhidgetImu::attached() const { return pimpl_->attached.load(std::memory_order_relaxed); }

bool PhidgetImu::read(ImuSample& out)
{
    std::scoped_lock lk(pimpl_->mtx);
    if (pimpl_->seq == pimpl_->last_read_seq)
        return false;                      // nothing new since last poll
    pimpl_->last_read_seq = pimpl_->seq;
    out = pimpl_->latest;
    return true;
}

std::string PhidgetImu::device_label() const
{
    std::scoped_lock lk(pimpl_->mtx);
    return pimpl_->label;
}
