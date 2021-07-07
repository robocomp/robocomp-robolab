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

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx, bool startup_check) : GenericWorker(mprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;

}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{

	timer.start(Period);
	laserConf.driver = params[LASER_DRIVER_PROPERTY_NAME].value;
	laserConf.device = params[LASER_DEVICE_PROPERTY_NAME].value;
	
	laserConf.iniRange = QString::fromStdString(params[LASER_START_PROPERTY_NAME].value).toInt();
	laserConf.endRange = QString::fromStdString(params[LASER_END_PROPERTY_NAME].value).toInt();
	
	laserConf.sampleRate = QString::fromStdString(params[LASER_SAMPLERATE_PROPERTY_NAME].value).toInt();
	laserConf.maxDegrees = QString::fromStdString(params[LASER_MAX_DEGREES_PROPERTY_NAME].value).toInt();
	laserConf.maxRange = QString::fromStdString(params[LASER_MAX_RANGE_PROPERTY_NAME].value).toInt();
	laserConf.minRange = QString::fromStdString(params[LASER_MIN_RANGE_PROPERTY_NAME].value).toInt();
	laserConf.staticConf = QString::fromStdString(params[LASER_STATIC_CONF_PROPERTY_NAME].value).toInt();
	laserConf.angleRes = QString::fromStdString(params[LASER_RESOLUTION_PROPERTY_NAME].value).toFloat();
	laserConf.angleIni = QString::fromStdString(params[LASER_INITIAL_ANGLE_PROPERTY_NAME].value).toFloat();
	laserConf.cluster = QString::fromStdString(params[LASER_CLUSTER_PROPERTY_NAME].value).toInt();	
	
	if ((laserConf.driver == "HokuyoURG")) // HOKUYO URG04LX
	{
        lh = new HokuyoHandler(laserConf, genericbase_proxy);
  	}
	else if ((laserConf.driver == "Hokuyo30LX") || (laserConf.driver == "HokuyoURG04LX-UG01"))
	{
		 lh = new HokuyoGenericHandler(laserConf, genericbase_proxy);
	}
	if ( !lh->open() )
	{
		qFatal( "[" PROGRAM_NAME "]: Unable to open device: %s", laserConf.device.c_str() );
	}
	qWarning( "[" PROGRAM_NAME "]: Device opened: %s", laserConf.device.c_str() );
	lh->start();
	rDebug("Hokuyo laser handler started");
	

	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	timer.start(Period);

}

void SpecificWorker::compute()
{
	QMutexLocker locker(mutex);
	
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


RoboCompLaser::TLaserData SpecificWorker::Laser_getLaserData()
{
	return lh->getNewData();
}

RoboCompLaser::LaserConfData SpecificWorker::Laser_getLaserConfData()
{
	return laserConf;
}

RoboCompLaser::TLaserData SpecificWorker::Laser_getLaserAndBStateData(RoboCompGenericBase::TBaseState &bState)
{
	bState = lh->getBaseState();
	return lh->getNewData();
}






