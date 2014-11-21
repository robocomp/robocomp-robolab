/*
 *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
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
#include "worker.h"
/**
* \brief Default constructor
*/
Worker::Worker(RoboCompDifferentialRobot::DifferentialRobotPrx differentialrobotprx, QObject *parent) : QObject(parent)
{
	differentialrobot = differentialrobotprx;


	mutex = new QMutex();
	//setupUi(this);
	//show();

	Period = BASIC_PERIOD;
}

/**
* \brief Default destructor
*/
Worker::~Worker()
{

}
void Worker::killYourSelf()
{
	rDebug("Killing myself");
	emit kill();
	exit(1);
}
/**
* \brief Change compute period
* @param per Period in ms
*/
void Worker::setPeriod(int p)
{
	rDebug("Period changed"+QString::number(p));
	Period = p;
}

void Worker::setParams(RoboCompCommonBehavior::ParameterList _params)
{
	laserConf.driver = _params[LASER_DRIVER_PROPERTY_NAME].value;
	laserConf.device = _params[LASER_DEVICE_PROPERTY_NAME].value;
	
	laserConf.iniRange = QString::fromStdString(_params[LASER_START_PROPERTY_NAME].value).toInt();
	laserConf.endRange = QString::fromStdString(_params[LASER_END_PROPERTY_NAME].value).toInt();
	
	laserConf.sampleRate = QString::fromStdString(_params[LASER_SAMPLERATE_PROPERTY_NAME].value).toInt();
	laserConf.maxDegrees = QString::fromStdString(_params[LASER_MAX_DEGREES_PROPERTY_NAME].value).toInt();
	laserConf.maxRange = QString::fromStdString(_params[LASER_MAX_RANGE_PROPERTY_NAME].value).toInt();
	laserConf.minRange = QString::fromStdString(_params[LASER_MIN_RANGE_PROPERTY_NAME].value).toInt();
	laserConf.staticConf = QString::fromStdString(_params[LASER_STATIC_CONF_PROPERTY_NAME].value).toInt();
	laserConf.angleRes = QString::fromStdString(_params[LASER_RESOLUTION_PROPERTY_NAME].value).toFloat();
	laserConf.angleIni = QString::fromStdString(_params[LASER_INITIAL_ANGLE_PROPERTY_NAME].value).toFloat();
	laserConf.maxMeasures = 600;
	laserConf.cluster = QString::fromStdString(_params[LASER_SKIP_PROPERTY_DEFAULT].value).toInt();	
	
	if ((laserConf.driver == "HokuyoURG")) // HOKUYO URG04LX
	{
		lh = new HokuyoHandler(laserConf, differentialrobot);
	}
	else if ((laserConf.driver == "Hokuyo30LX") || (laserConf.driver == "HokuyoURG04LX-UG01"))
	{
		#ifdef COMPILE_HOKUYO30LX
		 lh = new HokuyoGenericHandler(config, base_prx);
		#else
		 qFatal("LaserComp::LaserI::LaserI(): Config error: laserComp was not compiled with HOKUYO Laser support (please, install hokuyo library (sited in robocomp/Thirdparty). Exiting...");
		#endif
	}
	if ( !lh->open() )
	{
		qFatal( "[" PROGRAM_NAME "]: Unable to open device: %s", laserConf.device.c_str() );
	}

	qWarning( "[" PROGRAM_NAME "]: Device opened: %s", laserConf.device.c_str() );
	lh->start();
	rDebug("Hokuyo laser handler started");
}

RoboCompLaser::TLaserData Worker::getNewData()
{
	return lh->getNewData();
}
RoboCompLaser::LaserConfData Worker::getLaserConf()
{
	return laserConf;
}
RoboCompDifferentialRobot::TBaseState Worker::getBaseState()
{
	return lh->getBaseState();
}