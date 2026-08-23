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
#include <ranges>
#include <cstdint>
#include <cstdlib>
#include <print>

#include "dds_publisher.h"

#ifdef HAVE_PHIDGET22
#include "phidget_imu.h"
#endif

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

    // ---- Source gate: where the samples come from ----
    // Both sources produce the same ImuSample and share one publish path, so this only
    // changes where the data is READ, never what goes onto the plane.
    try { imu_source = this->configLoader.get<std::string>("IMU.Source"); }
    catch(...) { imu_source = "ice"; }
    std::ranges::transform(imu_source, imu_source.begin(), ::tolower);
    if (imu_source != "ice" and imu_source != "phidget")
    {
        std::cerr << "IMU.Source = '" << imu_source << "' is not one of ice|phidget — falling back to 'ice'."
                  << std::endl;
        imu_source = "ice";
    }
    use_phidget = (imu_source == "phidget");

    if (use_phidget)
    {
#ifdef HAVE_PHIDGET22
        PhidgetImu::Config pcfg;
        try { pcfg.data_interval_ms = this->configLoader.get<int>("Phidget.DataIntervalMs"); } catch(...) {}
        try { pcfg.serial = this->configLoader.get<int>("Phidget.Serial"); } catch(...) {}
        try { pcfg.hub_port = this->configLoader.get<int>("Phidget.HubPort"); } catch(...) {}
        try { pcfg.open_timeout_ms = this->configLoader.get<int>("Phidget.OpenTimeoutMs"); } catch(...) {}
        try { pcfg.use_ahrs = this->configLoader.get<bool>("Phidget.UseAHRS"); } catch(...) {}
        try { pcfg.gyro_var = static_cast<float>(this->configLoader.get<double>("Phidget.GyroVar")); } catch(...) {}
        try { pcfg.acc_var  = static_cast<float>(this->configLoader.get<double>("Phidget.AccVar")); } catch(...) {}

        // Poll at ~half the device period: the driver pushes on its own thread, so this only
        // sets how promptly compute() picks a sample up. Also the ceiling, so a missing device
        // cannot spin the loop.
        max_period_ms = std::max(1, pcfg.data_interval_ms / 2);

        phidget = std::make_unique<PhidgetImu>();
        if (phidget->start(pcfg))
            std::cout << "IMU source: phidget (" << phidget->device_label() << ")" << std::endl;
        else
            // Not fatal: compute() keeps retrying, so a cable plugged in later still works.
            std::cerr << "IMU source: phidget — no device attached yet; will keep retrying."
                      << std::endl;
#else
        std::cerr << "IMU.Source = 'phidget' but this build has NO Phidget support "
                     "(phidget22.h was not found at configure time). Install libphidget22-dev "
                     "and rebuild, or set IMU.Source = \"ice\"." << std::endl;
        use_phidget = false;
        imu_source = "ice";
#endif
    }

    if (not use_phidget)
    {
        std::cout << "IMU source: ice (" << (imu_proxy != nullptr ? "proxy ready" : "NO PROXY") << ")"
                  << std::endl;
        if (imu_proxy == nullptr)
            std::cerr << "No IMU proxy configured (Proxies.IMU) — nothing to publish." << std::endl;
    }
}


// Fill `out` from whichever source is configured. Everything downstream is source-agnostic.
bool SpecificWorker::read_sample(ImuSample& out)
{
    return use_phidget ? read_sample_phidget(out) : read_sample_ice(out);
}


bool SpecificWorker::read_sample_ice(ImuSample& out)
{
    if (imu_proxy == nullptr)
        return false;

    RoboCompIMU::DataImu data;
    try
    {
        data = imu_proxy->getDataImu();
    }
    catch (const Ice::Exception &e)
    {
        // Say it ONCE: at the sensor's own rate a per-failure message would be hundreds of
        // lines a second.
        if (not proxy_error_logged)
        {
            std::cerr << "Error reading from IMU: " << e.what() << " — retrying..." << std::endl;
            proxy_error_logged = true;
        }
        return false;
    }
    if (proxy_error_logged)
    {
        std::print("IMU stream recovered.\n");
        proxy_error_logged = false;
        last_imu_stamp_ms = 0;      // the gap is not a source period; don't feed it to the EMA
        imu_src_period_ms = -1.0;
    }

    // The acc substruct carries the freshest capture stamp; all substructs share a clock.
    out.stamp_ms = to_epoch_ms(data.acc.timestamp);
    out.acc[0]  = data.acc.XAcc;  out.acc[1]  = data.acc.YAcc;  out.acc[2]  = data.acc.ZAcc;
    out.gyro[0] = data.gyro.XGyr; out.gyro[1] = data.gyro.YGyr; out.gyro[2] = data.gyro.ZGyr;
    out.mag[0]  = data.mag.XMag;  out.mag[1]  = data.mag.YMag;  out.mag[2]  = data.mag.ZMag;
    out.rpy[0]  = data.rot.Roll;  out.rpy[1]  = data.rot.Pitch; out.rpy[2]  = data.rot.Yaw;
    out.temperature = data.temperature;
    // The two fields a media-plane consumer cannot reconstruct and must not have to guess.
    // simTimestamp is 0 on a real IMU, which is exactly the "not simulated" signal the
    // consumer needs; the gyro covariance is diagonal and isotropic, so m22 is its variance.
    out.sim_stamp_ms = static_cast<std::uint64_t>(std::max<long long>(0, data.gyro.simTimestamp));
    out.gyro_var     = data.gyro.cov.m22;
    // The accelerometer covariance is diagonal and isotropic like the gyro's; m00 is the
    // horizontal variance, which is the pair a consumer integrates for a velocity change.
    out.acc_var      = data.acc.cov.m00;
    return true;
}


bool SpecificWorker::read_sample_phidget(ImuSample& out)
{
#ifdef HAVE_PHIDGET22
    if (not phidget)
        return false;
    // No device yet (or it was unplugged and the handle never opened): retry the open on a
    // slow cadence rather than giving up, so plugging the IMU in brings the stream up with
    // no restart. Speak once per outage, not once per tick.
    if (not phidget->attached())
    {
        if (not phidget_retry_logged)
        {
            std::cerr << "Phidget IMU not attached — retrying..." << std::endl;
            phidget_retry_logged = true;
        }
        self_adjust_period(max_period_ms);
        return false;
    }
    if (phidget_retry_logged)
    {
        std::print("Phidget IMU attached: {}\n", phidget->device_label());
        phidget_retry_logged = false;
        last_imu_stamp_ms = 0;      // the outage is not a source period; don't feed the EMA
        imu_src_period_ms = -1.0;
    }
    return phidget->read(out);     // false = nothing new since the last poll
#else
    (void) out;
    return false;
#endif
}


void SpecificWorker::compute()
{
    if (not publish_dds)
    {
        self_adjust_period(max_period_ms);
        return;
    }

    // ONE path from here down, whichever source produced the sample. That is what makes the
    // stream on rc/imu/data identical for "ice" and "phidget".
    ImuSample s;
    if (not read_sample(s))
    {
        // No new sample: source down, or we polled faster than it produces. The ICE reader
        // wants the idle ceiling while its peer is refused; the Phidget reader is push-driven
        // and already paced by its DataInterval, so leave its period alone.
        if (not use_phidget)
            self_adjust_period(max_period_ms);
        return;
    }

    const std::uint64_t stamp_ms = s.stamp_ms;

    // Same sample as last tick: publishing it again would inflate the apparent rate and hand
    // a consumer a zero-dt pair.
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

    dds_publisher->publish(s);

    fps.print(use_phidget ? "Phidget->DDS" : "IMU->DDS", 3000);
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
