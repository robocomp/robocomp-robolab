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
Worker::Worker()
{
	mutex = new QMutex();
	period = 100;
	active = false;
	handler = NULL;	
}
/**
* \brief Default destructor
*/
Worker::~Worker()
{
	if(handler != NULL)
		delete handler;
	delete mutex;
}
/**
* \brief 
*/
void Worker::run( )
{
  forever
  {	
	handler->compute();
	this->usleep(period);
  }
}
/**
* \brief Change compute period
* @param per Period in ms
*/
void Worker::setPeriod(int per)
{
	rDebug("Period changed"+QString::number(per));
	mutex->lock();
		period = per;
	mutex->unlock();
}
/**
* \brief Return compute period in ms
* @return Compute period in ms
*/
int Worker::getPeriod()
{	
	QMutexLocker lock(mutex);
	return period;
}
/**
* \brief 
* @param _params List contains operation params.
*/
void Worker::setParams(RoboCompCommonBehavior::ParameterList _params)
{
	mutex->lock();
		active = false;
	mutex->unlock();
	if(_params["DRobot.Device"].value != params.device or _params["DRobot.Handler"].value != params.handler)
	{
		if(handler!= NULL)
			delete handler;
		
		params.device = _params["DRobot.Device"].value;
		params.handler = _params["DRobot.Handler"].value;
		params.maxVelAdv = QString::fromStdString(_params["DRobot.maxVelAdv"].value).toFloat();
		params.maxVelRot = QString::fromStdString(_params["DRobot.maxVelRot"].value).toFloat();
		//creacion del handler
		if ( params.handler == "Robex"){
			rInfo("Handler -> Robex");
			handler = new RobexHandler(params);
			params = handler->getMechParams();
			this->start();
		}
		else if ( params.handler == "Gazebo")
		#if COMPILE_GAZEBO == 1
		{
			rInfo("Handler -> Gazebo");
			handler = new GazeboHandler(params);
			params = handler->getMechParams();
			this->start();
		}
		#else
		{
			qFatal("Fatal error: differentialrobotComp was not compiled with Gazebo support. Exiting...");
			rError("Fatal error: differentialrobotComp was not compiled with Gazebo support. Exiting...");
			emit kill();
		}
		#endif
		else
		{
			qFatal( "No handler %s available. Aborting ",params.handler.c_str() );
			rError("No handler " +params.handler+" available. Aborting");
			emit kill();
		}
	}
	else
	{
		params.maxVelAdv = QString::fromStdString(_params["DRobot.maxVelAdv"].value).toFloat();
		params.maxVelRot = QString::fromStdString(_params["DRobot.maxVelRot"].value).toFloat();
	}
	mutex->lock();
		active = true;
	mutex->unlock();
}
/**
* \brief Set base speed, advance speed in mm/sg and turn speed in rads/sg
* @param adv Advance speed in mm/seg
* @param rot Turn speed in rads/seg
* @return true if command was executed successfully, else return false
*/

bool Worker::setSpeedBase(float adv ,float rot  )
{
	if(fabs(adv) > params.maxVelAdv )
		adv = params.maxVelAdv * adv/fabs(adv);
	if(fabs(rot) > params.maxVelRot)
		rot = params.maxVelRot * rot/fabs(rot);
	if (handler->setSpeedBase(adv,rot) )
	      return true;
	else
	      return true;
}
/**
* \brief Stops base
* @return true if command was executed successfully, else return false
*/
bool Worker::stopBase()
{
	return handler->stopBase();
}
/**
 * \brief Return current base state
 * @return bState Struct contains base position compute by odometry
 */
RoboCompDifferentialRobot::TBaseState Worker::getBaseState()
{
	return handler->getBaseState();
}
/**
 * \brief Return mechanical data from Handler.
 * @return TMechParams Struct contains mechanical params.
 */
RoboCompDifferentialRobot::TMechParams Worker::getMechParams()
{
	return params;
}
/**
 * \brief Reset odometer
 * @return true if command was sended successfully, else return false
 */
bool Worker::resetOdometer()
{
	return handler->resetOdometer();
}

void Worker::correctOdometer(float x, float z, float alpha)
{
	handler->correctOdometer(x, z, alpha);
}
/**
* \brief Set odometer to a specified state
* @param bState State to set odometer values
* @return true if command was sended successfully, else return false
*/
bool Worker::setOdometer(RoboCompDifferentialRobot::TBaseState bState)
{
	return handler->setOdometer(bState);
}
/**
* \brief Receive joystick data
* Transform data joystick in base speed
* @param data Data sended from joystick
*/
void Worker::processJoystickEvent(RoboCompJoystickAdapter::TData data)
{
	float adv = normalize(data.axes[data.velAxisIndex].value, -1, 1, -params.maxVelAdv, params.maxVelAdv);
	float rot = normalize(data.axes[data.dirAxisIndex].value, -1, 1, -params.maxVelRot, params.maxVelRot);
	handler->setSpeedBase(adv,rot);
}

float Worker::normalize(float X, float A, float B, float C, float D)
{
	return ((D-C)*(X-A)/(B-A))+C;
}