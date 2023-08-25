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


#define DEBUG 1
#define NAVDATA 0
#define GPSDATA 0

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>

// sbgCommonLib headers
#include <sbgCommon.h>
#include <version/sbgVersion.h>

// sbgECom headers
#include <sbgEComLib.h>

struct RoboCompSbgEkinox{        
  RoboCompGpsUblox::DatosGPS data_gps;     
  RoboCompIMU::DataImu data_imu;
} ; 
using RoboCompSbgEkinoxType = RoboCompSbgEkinox;

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:

	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	//Callback function 
    static SbgErrorCode callback(SbgEComHandle *pHandle, SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData *pLogData, void *pUserArg);
	//Show the data in variable data_imu
	void show_data(RoboCompSbgEkinoxType *_data_ekinox);

	RoboCompGpsUblox::DatosGPS GpsUblox_getData();
	void GpsUblox_setInitialPose(float x, float y);
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
	//SBG function
	SbgEComHandle init_sbg_ekinox(SbgInterface *pInterface, RoboCompSbgEkinoxType *_data_ekinox);
	SbgErrorCode print_product_info(SbgEComHandle *pECom);
	//SBG Variables
	SbgInterface sbgInterface;
	SbgEComHandle comHandle;

	bool startup_check_flag;
	RoboCompSbgEkinox data_ekinox;
};
#endif
