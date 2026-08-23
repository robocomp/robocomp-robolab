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

// ImuSample — the ONE currency this component deals in.
//
// Every source fills this struct and the publisher consumes it, so the bytes on
// rc/imu/data are produced by a single code path no matter where the data came from.
// That is the whole point of the IMU.Source gate: a consumer must not be able to tell
// a Webots-backed run from a Phidget-backed one by the shape or units of the stream.
//
// UNITS ARE SI AND NON-NEGOTIABLE — m/s^2, rad/s, rad. A source that speaks anything
// else (the Phidget driver reports g and deg/s) converts on the way in, never on the
// way out. See imu_frame.idl, which documents the same contract for the wire type.

#include <cstdint>

struct ImuSample
{
    std::uint64_t stamp_ms = 0;        // capture time, WALL epoch ms

    // Producer's SIMULATION clock in ms; 0 when the source is a real sensor, which is
    // exactly the "not simulated" flag a consumer needs. A simulator reports rates per
    // SIMULATION second while its stamps are wall, so anything integrating the gyro must
    // key on the clock the rate is measured against.
    std::uint64_t sim_stamp_ms = 0;

    float acc[3]  = {0.f, 0.f, 0.f};   // linear acceleration, m/s^2 (gravity included)
    float gyro[3] = {0.f, 0.f, 0.f};   // angular velocity, rad/s
    float mag[3]  = {0.f, 0.f, 0.f};   // magnetic field, Gauss (NaN when unavailable)
    float rpy[3]  = {0.f, 0.f, 0.f};   // roll, pitch, yaw, rad
    float temperature = 0.f;           // degC, 0 when the source carries no temperature

    // Per-sample variances. NEGATIVE means "the producer does not know" — fill that
    // rather than 0, which reads downstream as infinite confidence.
    float gyro_var = -1.f;             // (rad/s)^2
    float acc_var  = -1.f;             // (m/s^2)^2
};
