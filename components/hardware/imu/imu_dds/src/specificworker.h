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

/**
	\brief
	@author authorname
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

// NOT enabled on purpose. Hibernation drops the Compute period to 500 ms when no ICE
// method is called for 5 s, and this component's only ICE method is the one-shot
// descriptor query — so the sensor pump would throttle itself to 2 Hz precisely while
// it was working correctly on the DDS plane.
//#define HIBERNATION_ENABLED

#include <genericworker.h>

#include <cstdint>
#include <memory>

// Zero-copy DDS media-plane publisher (ImuFrame), gated by config. FastDDS headers stay
// behind this forward declaration (PIMPL in dds_publisher.cpp).
class ImuDDSPublisher;

/**
 * \brief Class SpecificWorker implements the core functionality of the component.
 *
 * imu_dds is a one-way bridge: it pulls IMU samples off the ICE IMU interface (the
 * Webots bridge in simulation, a Phidget/SBG driver on hardware) and republishes them
 * on the CORTEX zero-copy DDS media plane (domain 7, topic rc/imu/data), advertising
 * the plane over MediaPlaneDDS so robot_concept can relay the descriptor into DSR.
 */
class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
    /**
     * \brief Constructor for SpecificWorker.
     * \param configLoader Configuration loader for the component.
     * \param tprx Tuple of proxies required for the component.
     * \param startup_check Indicates whether to perform startup checks.
     */
	SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);

	/**
     * \brief Destructor for SpecificWorker.
     */
	~SpecificWorker();

	std::string MediaPlaneDDS_getMediaDescriptor();


public slots:

	/**
	 * \brief Initializes the worker one time.
	 */
	void initialize();

	/**
	 * \brief Main compute loop of the worker.
	 */
	void compute();

	/**
	 * \brief Handles the emergency state loop.
	 */
	void emergency();

	/**
	 * \brief Restores the component from an emergency state.
	 */
	void restore();

    /**
     * \brief Performs startup checks for the component.
     * \return An integer representing the result of the checks.
     */
	int startup_check();

private:

	/**
     * \brief Flag indicating whether startup checks are enabled.
     */
	bool startup_check_flag;

	// --- Zero-copy DDS imu media plane (null unless PublishDDS is enabled) ---
	bool publish_dds = false;
	std::unique_ptr<ImuDDSPublisher> dds_publisher;

	// --- Self-synchronization with the source ---
	// The IMU runs far faster than any sensible fixed Compute period (~125 Hz in Webots),
	// so the period is driven by the SOURCE's own stamp deltas rather than configured:
	// poll at ~half the measured source period, and drop samples whose stamp we already
	// published. Pacing on our own wall-clock loop timing instead would be circular and
	// death-spirals the rate downwards.
	std::uint64_t last_imu_stamp_ms = 0;   // dedup key: last published source stamp
	double imu_src_period_ms = -1.0;       // EMA of the source period, from stamp deltas
	int max_period_ms = 100;               // idle/failure ceiling (config Period.Compute)
	int min_period_ms = 1;                 // floor: never busy-spin the proxy
	bool proxy_error_logged = false;       // one-shot: don't spam while the source is down

	// Move the Compute period towards `target_ms`. Unlike lidar3d_dds's ±1 ms ramp this
	// jumps straight there: the IMU's period is measured from the source itself, so the
	// target is already right, and ramping one millisecond per tick would take seconds to
	// climb down from the 100 ms default to the 4 ms the sensor actually needs.
	void self_adjust_period(int target_ms);

	// Normalize a source timestamp to epoch ms. Producers disagree: the Webots bridge
	// reports ms, some hardware drivers report ns. A value past ~1e15 cannot be ms (that
	// is the year 33658), so it is ns. Same rule robot_concept's readers use.
	static std::uint64_t to_epoch_ms(long long t)
	{
		return t > 1'000'000'000'000'000LL
		           ? static_cast<std::uint64_t>(t / 1'000'000)   // ns -> ms
		           : static_cast<std::uint64_t>(t);              // already ms
	}

signals:
	//void customSignal();
};

#endif
