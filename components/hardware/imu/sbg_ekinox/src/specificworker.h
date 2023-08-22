/*
 *    Copyright (C) 2023 by YOUR NAME HERE
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

#include <genericworker.h>

// sbgCommonLib headers
#include <sbgCommon.h>
#include <version/sbgVersion.h>

// sbgECom headers
#include <sbgEComLib.h>

class CallbackHandler {
private:
    float angle = 0.0;

public:
    SbgErrorCode onLogReceived(SbgEComHandle *pHandle, SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData *pLogData, void *pUserArg) {
        assert(pLogData);

        SBG_UNUSED_PARAMETER(pHandle);
        SBG_UNUSED_PARAMETER(pUserArg);

        if (msgClass == SBG_ECOM_CLASS_LOG_ECOM_0) {
            switch (msg) {
                case SBG_ECOM_LOG_EKF_EULER:
                    printf("Euler Angles: %3.1f\t%3.1f\t%3.1f\tStd Dev:%3.1f\t%3.1f\t%3.1f   \r",
                           sbgRadToDegf(pLogData->ekfEulerData.euler[0]),
                           sbgRadToDegf(pLogData->ekfEulerData.euler[1]),
                           sbgRadToDegf(pLogData->ekfEulerData.euler[2]),
                           sbgRadToDegf(pLogData->ekfEulerData.eulerStdDev[0]),
                           sbgRadToDegf(pLogData->ekfEulerData.eulerStdDev[1]),
                           sbgRadToDegf(pLogData->ekfEulerData.eulerStdDev[2]));
                    angle = sbgRadToDegf(pLogData->ekfEulerData.euler[0]);
                    break;
                default:
                    break;
            }
        }

        return SBG_NO_ERROR;
    }

    float getAngle() const {
        return angle;
    }
};

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	static CallbackHandler s_handler;  // Variable estática

    static SbgErrorCode onLogReceived(SbgEComHandle *pHandle, SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData *pLogData, void *pUserArg);
	static SbgErrorCode ellipseMinimalProcess(SbgInterface *pInterface);
	static SbgErrorCode getAndPrintProductInfo(SbgEComHandle *pECom);


	RoboCompIMU::Acceleration IMU_getAcceleration();
	RoboCompIMU::Gyroscope IMU_getAngularVel();
	RoboCompIMU::DataImu IMU_getDataImu();
	RoboCompIMU::Magnetic IMU_getMagneticFields();
	RoboCompIMU::Orientation IMU_getOrientation();
	void IMU_resetImu();


public slots:
	void compute();
	int startup_check();
	void initialize(int period);
private:
	bool startup_check_flag;
	SbgInterface sbgInterface;
	std::string rs232, IP_address;
	int baudrate, input_port, output_port;
	float angle;



};

#endif
