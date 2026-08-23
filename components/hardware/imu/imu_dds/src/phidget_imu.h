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

#pragma once

// PhidgetImu — reads a real Phidget Spatial (Precision 3/3/3 and friends) through the
// native phidget22 C library, and hands out ImuSamples in the SAME SI units the ICE
// source produces. It is the second of the two interchangeable sources behind the
// IMU.Source config gate; the publish path downstream is identical for both, so the
// bytes on rc/imu/data do not reveal which one is running.
//
// THREADING. The phidget22 driver delivers samples on ITS OWN thread, at the configured
// DataInterval — it is push, not poll, unlike the ICE source. This class absorbs that:
// the callback stores the newest sample under a mutex and bumps a sequence counter, and
// read() copies it out from the compute() thread, reporting false when nothing new has
// arrived. So compute() stays a plain polling loop and never blocks on the device.
//
// Only the LATEST sample is kept, deliberately. This component republishes a live sensor
// onto a real-time media plane whose QoS is KEEP_LAST — a consumer that fell behind wants
// the freshest sample, not a backlog. (imu_fusion's ESKF is the case that genuinely needs
// every sample, and it does its own buffering in phidget.py.)
//
// phidget22.h stays behind a PIMPL so it never reaches the rest of the component.

#include <memory>
#include <string>

#include "imu_sample.h"

class PhidgetImu
{
public:
    struct Config
    {
        // Sampling period asked of the device, ms. Clamped up to the device's own
        // MinDataInterval on attach. 8 ms = 125 Hz, matching the simulated IMU's rate so
        // a consumer sees the same cadence on either source.
        int data_interval_ms = 8;
        // Device selection. -1 = "any", which is right for the usual single-IMU robot.
        int serial   = -1;
        int hub_port = -1;
        // How long start() waits for the device to attach before giving up.
        int open_timeout_ms = 5000;
        // Take orientation from the device's on-board AHRS (quaternion -> roll/pitch/yaw).
        // With this off, rpy is left at 0 rather than guessed: a 6-axis read cannot observe
        // yaw at all, and a fabricated one is worse than an absent one.
        bool use_ahrs = true;
        // Nominal per-sample variances published alongside the data, SI^2. The datasheet
        // figure is the honest default for a device that reports no covariance of its own;
        // negative means "unknown", which is what a consumer must see if you cannot state one.
        float gyro_var = 1e-5f;    // (rad/s)^2
        float acc_var  = 1e-3f;    // (m/s^2)^2
    };

    PhidgetImu();
    ~PhidgetImu();
    PhidgetImu(const PhidgetImu&) = delete;
    PhidgetImu& operator=(const PhidgetImu&) = delete;

    // Open the device and wait (bounded) for attach. False if no device showed up — the
    // caller should treat that as "source down", not as a fatal error, so a cable can be
    // plugged in later. Safe to call again after a failure.
    bool start(const Config& cfg);
    void stop();

    // True while the driver reports the device attached (cleared by the detach handler,
    // set again on re-attach — the phidget22 library reconnects on its own).
    [[nodiscard]] bool attached() const;

    // Copy out the newest sample. False when nothing new has arrived since the last call,
    // which is the normal case whenever compute() polls faster than DataInterval.
    bool read(ImuSample& out);

    // Human-readable device id ("Spatial serial 123456 hub port 0"), for logging. Empty
    // until attached.
    [[nodiscard]] std::string device_label() const;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};
