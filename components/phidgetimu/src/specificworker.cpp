/*
 *    Copyright (C) 2016 by YOUR NAME HERE
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

float Acc[3];
float Gyr[3];
float Mag[3];
float Ori[3];


//callback that will run if the Spatial is attached to the computer
int CCONV AttachHandler(CPhidgetHandle spatial, void *userptr)
{
	int serialNo;
	CPhidget_getSerialNumber(spatial, &serialNo);
	printf("Spatial %10d attached!", serialNo);
	return 0;
}

//callback that will run if the Spatial is detached from the computer
int CCONV DetachHandler(CPhidgetHandle spatial, void *userptr)
{
	int serialNo;
	CPhidget_getSerialNumber(spatial, &serialNo);
	printf("Spatial %10d detached! \n", serialNo);
	return 0;
}

//callback that will run if the Spatial generates an error
int CCONV ErrorHandler(CPhidgetHandle spatial, void *userptr, int ErrorCode, const char *unknown)
{
	printf("Error handled. %d - %s \n", ErrorCode, unknown);
	return 0;
}

//callback that will run at datarate
//data - array of spatial event data structures that holds the spatial data packets that were sent in this event
//count - the number of spatial data event packets included in this event
int CCONV SpatialDataHandler(CPhidgetSpatialHandle spatial, void *userptr, CPhidgetSpatial_SpatialEventDataHandle *data, int count)
{
	Acc[0] = -data[count-1]->acceleration[0];
	Acc[1] = -data[count-1]->acceleration[2];
	Acc[2] =  data[count-1]->acceleration[1];
	Gyr[0] = -data[count-1]->angularRate[0]*M_PI/180.;
	Gyr[1] = -data[count-1]->angularRate[2]*M_PI/180.;
	Gyr[2] =  data[count-1]->angularRate[1]*M_PI/180.;
	Mag[0] = -data[count-1]->magneticField[0];
	Mag[1] = -data[count-1]->magneticField[2];
	Mag[2] =  data[count-1]->magneticField[1];

	
	Ori[0] =  atan2(Acc[2], -Acc[1]);
	Ori[1] = 0;
	Ori[2] = -atan2(Acc[0], -Acc[1]);
	for (int o=0; o<3; o++)
	{
		while(Ori[o] > +M_PI) Ori[o]-=2.*M_PI;
		while(Ori[o] < -M_PI) Ori[o]+=2.*M_PI;
	}
// 	printf("Acc: (%f, %f, %f)\n", Acc[0], Acc[1], Acc[2]);
// 	printf("Ori: (%f, %f, %f)\n", Ori[0], Ori[1], Ori[2]);
	return 0;
}


//Display the properties of the attached phidget to the screen.  
//We will be displaying the name, serial number, version of the attached device, the number of accelerometer, gyro, and compass Axes, and the current data rate
// of the attached Spatial.
int display_properties(CPhidgetHandle phid)
{
	int serialNo, version;
	const char* ptr;
	int numAccelAxes, numGyroAxes, numCompassAxes, dataRateMax, dataRateMin;

	CPhidget_getDeviceType(phid, &ptr);
	CPhidget_getSerialNumber(phid, &serialNo);
	CPhidget_getDeviceVersion(phid, &version);
	CPhidgetSpatial_getAccelerationAxisCount((CPhidgetSpatialHandle)phid, &numAccelAxes);
	CPhidgetSpatial_getGyroAxisCount((CPhidgetSpatialHandle)phid, &numGyroAxes);
	CPhidgetSpatial_getCompassAxisCount((CPhidgetSpatialHandle)phid, &numCompassAxes);
	CPhidgetSpatial_getDataRateMax((CPhidgetSpatialHandle)phid, &dataRateMax);
	CPhidgetSpatial_getDataRateMin((CPhidgetSpatialHandle)phid, &dataRateMin);

	

	printf("%s\n", ptr);
	printf("Serial Number: %10d\nVersion: %8d\n", serialNo, version);
	printf("Number of Accel Axes: %i\n", numAccelAxes);
	printf("Number of Gyro Axes: %i\n", numGyroAxes);
	printf("Number of Compass Axes: %i\n", numCompassAxes);
	printf("datarate> Max: %d  Min: %d\n", dataRateMax, dataRateMin);

	return 0;
}

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	int result;
	const char *err;

	CPhidgetSpatialHandle spatial = 0;
	CPhidgetSpatial_create(&spatial);
	CPhidget_open((CPhidgetHandle)spatial, -1);
	if((result = CPhidget_waitForAttachment((CPhidgetHandle)spatial, 3000)))
	{
		CPhidget_getErrorDescription(result, &err);
		printf("Problem waiting for attachment: %s\n", err);
		exit(1);
	}
	display_properties((CPhidgetHandle)spatial);
	CPhidgetSpatial_setDataRate(spatial, 16);
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)spatial, AttachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)spatial, DetachHandler, NULL);
	CPhidget_set_OnError_Handler((CPhidgetHandle)spatial, ErrorHandler, NULL);
	CPhidgetSpatial_set_OnSpatialData_Handler(spatial, SpatialDataHandler, NULL);
	printf("Reading.....\n");
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	

	return true;
}

void SpecificWorker::compute()
{
// 	try
// 	{
// 		camera_proxy->getYImage(0,img, cState, bState);
// 		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
// 		searchTags(image_gray);
// 	}
// 	catch(const Ice::Exception &e)
// 	{
// 		std::cout << "Error reading from Camera" << e << std::endl;
// 	}
}

void SpecificWorker::resetImu()
{
	mutex->lock();
	for (int i=0; i<3; ++i)
	{
		Mag[i] = 0.f;
		Acc[i] = 0.f;
		Gyr[i] = 0.f;
	}
	mutex->unlock();
}

Gyroscope SpecificWorker::getAngularVel()
{
	Gyroscope d;
	mutex->lock();
	d.XGyr = Gyr[0];
	d.YGyr = Gyr[1];
	d.ZGyr = Gyr[2];
	mutex->unlock();
	return d;
}

Orientation SpecificWorker::getOrientation()
{
	Orientation d;
	mutex->lock();
	d.Pitch = Ori[0];
	d.Yaw   = Ori[1];
	d.Roll  = Ori[2];
	mutex->unlock();
	return d;
}

DataImu SpecificWorker::getDataImu()
{
	DataImu d;
	d.acc = getAcceleration();
	d.gyro = getAngularVel();
	d.mag = getMagneticFields();
	d.rot = getOrientation();
	return d;
}

Magnetic SpecificWorker::getMagneticFields()
{
	Magnetic d;
	mutex->lock();
	d.XMag = Mag[0];
	d.YMag = Mag[1];
	d.ZMag = Mag[2];
	mutex->unlock();
	return d;
}

Acceleration SpecificWorker::getAcceleration()
{
	Acceleration d;
	mutex->lock();
	d.XAcc = Acc[0];
	d.YAcc = Acc[1];
	d.ZAcc = Acc[2];
	printf("acc: %f %f %f\n", d.XAcc, d.YAcc, d.ZAcc);
	mutex->unlock();
	return d;
}






