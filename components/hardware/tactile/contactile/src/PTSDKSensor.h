#ifndef PTSDKSENSOR_H
#define PTSDKSENSOR_H
#endif

#ifdef _WIN32
#ifndef PTSDK_CPP_LIB_H
#include "PTSDK_CPP_LIB.h"
#endif
#endif

#ifndef PTSDKCONSTANTS_H
#include "PTSDKConstants.h"
#endif

#include <stdint.h>

/** \brief The PTSDKSensor class describes a PapillArray Tactile Sensor comprised of multiple pillars.
 *
 * The PTSDKSensor class describes a PapillArray sensor comprised of multiple pillars.
 *
 * @author Contactile Pty Ltd
 * @version July 2021
 */
#ifdef _WIN32
class PTSDK_CPP_LIB_API PTSDKSensor {
#else
class PTSDKSensor {
#endif

	friend class PTSDKListener;

private:
	/* Private member variables of PTSDKSensor class */

	uint64_t timestamp_us;
	double globalForceVals[NDIM];		// Buffer of 3D force values
	double globalTorqueVals[NDIM];		// Buffer of 3D torque values


	/* private member functions of the PTSDKSensor class */

	void addSensorSample(IN const float globalForceVals[NDIM], IN const float globalTorqueVals[NDIM], IN const uint64_t timestamp_us);

public:

	/**
	 * Constructor - Initialises pillars and their calibrations.
	 *
	 * @param[in] calibrationFilename The name of the calibration file.
	 */
	PTSDKSensor(void);

	/**
	 * Destructor.
	 */
	~PTSDKSensor(void);

	/**
	 * Gets the global X,Y,Z force (N) acting on the sensor.
	 *
	 * @param[out] result The global X, Y and Z force (N).
	 */
	void getGlobalForce(OUT double result[NDIM]);

	/**
	 * Gets the global X,Y,Z torque (N.mm) acting on the sensor.
	 *
	 * @param[out] result The global X, Y and Z torque (N.mm).
	 */
	void getGlobalTorque(OUT double result[NDIM]);

	/**
	 * Gets the current timestamp in us.
	 *
	 * @return The current timestamp in us.
	 */
	uint64_t getTimestamp_us(void);

};
