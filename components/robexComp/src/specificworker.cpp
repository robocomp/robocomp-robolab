/*
 *    Copyright (C)2019 by YOUR NAME HERE
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
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList _params)
{
	params.device = _params["DRobot.Device"].value;
	params.handler = _params["DRobot.Handler"].value;
	params.maxVelAdv = QString::fromStdString(_params["DRobot.maxVelAdv"].value).toFloat();
	params.maxVelRot = QString::fromStdString(_params["DRobot.maxVelRot"].value).toFloat();
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	if(handler!= NULL)
		delete handler;
	//creacion del handler
	if ( params.handler == "Robex"){
		rInfo("Handler -> Robex");
		handler = new RobexHandler(params);
		params = handler->getMechParams();
	}
	else if ( params.handler == "Gazebo")
	#if COMPILE_GAZEBO == 1
	{
		rInfo("Handler -> Gazebo");
		handler = new GazeboHandler(params);
		params = handler->getMechParams();
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
	
	this->Period = period;
	timer.start(Period);
}

void SpecificWorker::compute()
{
	handler->compute();
}


void SpecificWorker::DifferentialRobot_correctOdometer(const int x, const int z, const float alpha)
{
	handler->correctOdometer(x, z, alpha);
}

void SpecificWorker::DifferentialRobot_getBasePose(int &x, int &z, float &alpha)
{
	x = bState.x;
	z = bState.z;
	alpha = bState.alpha;
}

void SpecificWorker::DifferentialRobot_resetOdometer()
{
	handler->resetOdometer();
}

void SpecificWorker::DifferentialRobot_setOdometer(const RoboCompGenericBase::TBaseState &state)
{
	handler->setOdometer(state);
}

void SpecificWorker::DifferentialRobot_getBaseState(RoboCompGenericBase::TBaseState &state)
{
	state = handler->getBaseState();
}

void SpecificWorker::DifferentialRobot_setOdometerPose(const int x, const int z, const float alpha)
{
	RoboCompGenericBase::TBaseState bState;
	bState.correctedX     = bState.x     = x;
	bState.correctedZ     = bState.z     = z;
	bState.correctedAlpha = bState.alpha = alpha;
	this->DifferentialRobot_setOdometer(bState);
}

void SpecificWorker::DifferentialRobot_stopBase()
{
	handler->stopBase();
}

void SpecificWorker::DifferentialRobot_setSpeedBase(const float adv, const float rot)
{
	float adv2 = adv;
	float rot2 = rot;
	if(fabs(adv) > params.maxVelAdv )
		adv2 = params.maxVelAdv * adv/fabs(adv);
	if(fabs(rot) > params.maxVelRot)
		rot2 = params.maxVelRot * rot/fabs(rot);
	handler->setSpeedBase(adv2, rot2);
}

void SpecificWorker::GenericBase_getBaseState(RoboCompGenericBase::TBaseState &state)
{
	state = handler->getBaseState();
}

void SpecificWorker::GenericBase_getBasePose(int &x, int &z, float &alpha)
{
	x = bState.x;
	z = bState.z;
	alpha = bState.alpha;
}

void SpecificWorker::JoystickAdapter_sendData(const TData &data)
{
	float adv = normalize(data.axes[data.velAxisIndex].value, -1, 1, -params.maxVelAdv, params.maxVelAdv);
	float rot = normalize(data.axes[data.dirAxisIndex].value, -1, 1, -params.maxVelRot, params.maxVelRot);
	handler->setSpeedBase(adv,rot);
}

//UTILITIES
float SpecificWorker::normalize(float X, float A, float B, float C, float D)
{
	return ((D-C)*(X-A)/(B-A))+C;
}

