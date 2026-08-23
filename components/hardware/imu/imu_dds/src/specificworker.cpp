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
#include "specificworker.h"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <print>

#include "dds_publisher.h"

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
	this->startup_check_flag = startup_check;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif

		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0){
			qWarning() << error;
			throw error;
		}
	}
}

SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}


void SpecificWorker::initialize()
{
    std::cout << "initialize worker" << std::endl;
	GenericWorker::initialize();

    // The configured Compute period is only the CEILING here (the idle / source-down rate).
    // compute() drives the real period down from it once it has measured the source, so a
    // slow configured value can never cap the IMU the way a fixed period would.
    max_period_ms = std::max(1, getPeriod("Compute"));

    // ---- Zero-copy DDS imu media plane (ImuFrame into the CORTEX stack) ----
    // Gated by PublishDDS. Metadata is advertised into DSR by robot_concept (which pulls
    // it through MediaPlaneDDS_getMediaDescriptor below), not here — this component is
    // not a DSR agent and never joins the graph.
    try { publish_dds = this->configLoader.get<bool>("PublishDDS"); }
    catch(...) { publish_dds = false; }
    if (publish_dds)
    {
        ImuDDSPublisher::Config dcfg;   // defaults: domain 7, rc/imu/data
        try { dcfg.domain_id = static_cast<std::uint32_t>(this->configLoader.get<int>("DDS.Domain")); } catch(...) {}
        try { dcfg.topic = this->configLoader.get<std::string>("DDS.Topic"); } catch(...) {}
        try { dcfg.history_depth = this->configLoader.get<int>("DDS.HistoryDepth"); } catch(...) {}
        try { dcfg.shared_memory_only = this->configLoader.get<bool>("DDS.SharedMemoryOnly"); } catch(...) {}
        try { dcfg.data_sharing = this->configLoader.get<bool>("DDS.DataSharing"); } catch(...) {}

        dds_publisher = std::make_unique<ImuDDSPublisher>();
        if (not dds_publisher->init(dcfg))
        {
            std::cerr << "DDS imu media plane failed to initialize; disabling DDS publishing." << std::endl;
            dds_publisher.reset();
            publish_dds = false;
        }
    }
    else
        std::cout << "PublishDDS is false — no media plane will be created." << std::endl;

    if (imu_proxy == nullptr)
        std::cerr << "No IMU proxy configured (Proxies.IMU) — nothing to publish." << std::endl;
}


void SpecificWorker::compute()
{
    if (imu_proxy == nullptr or not publish_dds)
    {
        self_adjust_period(max_period_ms);
        return;
    }

    RoboCompIMU::DataImu data;
    try
    {
        data = imu_proxy->getDataImu();
    }
    catch (const Ice::Exception &e)
    {
        // Back off to the idle ceiling while the source is down, and say so ONCE: at the
        // sensor's own rate a per-failure message would be hundreds of lines a second.
        if (not proxy_error_logged)
        {
            std::cerr << "Error reading from IMU: " << e.what() << " — retrying..." << std::endl;
            proxy_error_logged = true;
        }
        self_adjust_period(max_period_ms);
        return;
    }
    if (proxy_error_logged)
    {
        std::print("IMU stream recovered.\n");
        proxy_error_logged = false;
        last_imu_stamp_ms = 0;      // the gap is not a source period; don't feed it to the EMA
        imu_src_period_ms = -1.0;
    }

    // The acc substruct carries the freshest capture stamp; all substructs share a clock,
    // so it is the frame timestamp and the dedup key.
    const std::uint64_t stamp_ms = to_epoch_ms(data.acc.timestamp);

    // Same sample as last tick: we polled faster than the source produces. Publishing it
    // again would inflate the apparent rate and hand a consumer a zero-dt pair.
    if (stamp_ms != 0 and stamp_ms == last_imu_stamp_ms)
        return;

    // Measure the SOURCE period from its own stamps and poll at ~2x it. Bounds reject a
    // stalled clock (dt 0) and a restart/rollover (dt seconds) — neither is a period.
    if (stamp_ms != 0)
    {
        if (last_imu_stamp_ms != 0)
            if (const double src_dt = static_cast<double>(stamp_ms - last_imu_stamp_ms);
                src_dt > 0.5 and src_dt < 2000.0)
            {
                imu_src_period_ms = (imu_src_period_ms < 0.0) ? src_dt
                                                              : 0.8 * imu_src_period_ms + 0.2 * src_dt;
                self_adjust_period(static_cast<int>(0.5 * imu_src_period_ms + 0.5));
            }
        last_imu_stamp_ms = stamp_ms;
    }

    ImuDDSPublisher::Sample s;
    s.stamp_ms = stamp_ms;
    s.acc[0]  = data.acc.XAcc;  s.acc[1]  = data.acc.YAcc;  s.acc[2]  = data.acc.ZAcc;
    s.gyro[0] = data.gyro.XGyr; s.gyro[1] = data.gyro.YGyr; s.gyro[2] = data.gyro.ZGyr;
    s.mag[0]  = data.mag.XMag;  s.mag[1]  = data.mag.YMag;  s.mag[2]  = data.mag.ZMag;
    s.rpy[0]  = data.rot.Roll;  s.rpy[1]  = data.rot.Pitch; s.rpy[2]  = data.rot.Yaw;
    s.temperature = data.temperature;
    // The two fields a media-plane consumer cannot reconstruct and must not have to guess.
    // simTimestamp is 0 on a real IMU, which is exactly the "not simulated" signal the
    // consumer needs; the gyro covariance is diagonal and isotropic, so m22 is its variance.
    s.sim_stamp_ms = static_cast<std::uint64_t>(std::max<long long>(0, data.gyro.simTimestamp));
    s.gyro_var     = data.gyro.cov.m22;
    // The accelerometer covariance is diagonal and isotropic like the gyro's; m00 is the
    // horizontal variance, which is the pair a consumer integrates for a velocity change.
    s.acc_var      = data.acc.cov.m00;

    dds_publisher->publish(s);

    fps.print("IMU->DDS", 3000);
}


void SpecificWorker::self_adjust_period(int target_ms)
{
    const int clamped = std::clamp(target_ms, min_period_ms, max_period_ms);
    // 1 ms deadband (as in lidar3d_dds). Half a ~8.7 ms source period lands between two
    // integer milliseconds, so without this the period flaps 4<->5 ms on every single tick
    // and the state machine logs a line for each — hundreds a second at IMU rates. Being
    // 1 ms fast is free: the stamp dedup in compute() drops the extra poll.
    if (std::abs(clamped - getPeriod("Compute")) < 2)
        return;
    setPeriod("Compute", clamped);
}


void SpecificWorker::emergency()
{
    fps.print("Emergency worker", 3000);
    //emergencyCODE
    //
    //if (SUCCESSFUL) //The componet is safe for continue
    //  emmit goToRestore()
}


//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
    //restoreCODE
    //Restore emergency component

}


int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
}

std::string SpecificWorker::MediaPlaneDDS_getMediaDescriptor()
{
	// Report the live DDS imu media-plane descriptor (JSON) so the robot_concept agent can
	// relay it onto the "imu" DSR node. Empty when DDS publishing is disabled/failed, which
	// is what tells robot_concept to keep bridging the IMU over Ice itself.
	if (dds_publisher)
		return dds_publisher->descriptor_json();
	return {};
}



/**************************************/
// From the RoboCompIMU you can call this methods:
// RoboCompIMU::Acceleration this->imu_proxy->getAcceleration()
// RoboCompIMU::Gyroscope this->imu_proxy->getAngularVel()
// RoboCompIMU::DataImu this->imu_proxy->getDataImu()
// RoboCompIMU::Magnetic this->imu_proxy->getMagneticFields()
// RoboCompIMU::Orientation this->imu_proxy->getOrientation()
// RoboCompIMU::void this->imu_proxy->resetImu()

/**************************************/
// From the RoboCompIMU you can use this types:
// RoboCompIMU::Cov3
// RoboCompIMU::Acceleration
// RoboCompIMU::Gyroscope
// RoboCompIMU::Magnetic
// RoboCompIMU::Orientation
// RoboCompIMU::DataImu
