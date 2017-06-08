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
#include "differentialrobotI.h"

DifferentialRobotI::DifferentialRobotI(Worker *_worker, QObject *parent) : QObject(parent)
{
	worker = _worker;
	mutex = worker->mutex;       // Shared worker mutex
	// Component initialization...
}


DifferentialRobotI::~DifferentialRobotI()
{
	// Free component resources here
}

// Component functions, implementation

void DifferentialRobotI::getBaseState(RoboCompGenericBase::TBaseState &bState, const Ice::Current&) 
{ 
	mutex->lock();
	if(worker->active)
	{
		mutex->unlock();
		bState = worker->getBaseState();
	}
	else
	{
		mutex->unlock();
		throwException("Exception: DifferentialRobotComp::Worker::getBaseState:: worker is sttoped by setting");
	}
}
void DifferentialRobotI::getBasePose(int &x, int &z, float &alpha, const Ice::Current&) 
{ 
	mutex->lock();
	if(worker->active)
	{
		mutex->unlock();
		RoboCompGenericBase::TBaseState bState = worker->getBaseState();
		x=bState.x;
		z=bState.z;
		alpha=bState.alpha;
	}
	else
	{
		mutex->unlock();
		throwException("Exception: DifferentialRobotComp::Worker::getBaseStatePose:: worker is sttoped by setting");
	}
}
void DifferentialRobotI::setSpeedBase(float adv, float rot, const Ice::Current&) 
{ 
	mutex->lock();
	if(worker->active)
	{
		mutex->unlock();
		if(!worker->setSpeedBase( adv , rot  ))
			throwException("Exception: DifferentialRobotComp::Worker::setSpeedBase:: Failure comunication exception");
	}
	else
	{
		mutex->unlock();
		throwException("Exception: DifferentialRobotComp::Worker::setSpeedBase:: Worker is sttoped by setting");
	}
}
void DifferentialRobotI::stopBase( const Ice::Current&) 
{ 
	mutex->lock();
	if(worker->active)
	{
		mutex->unlock();
		if(!worker->stopBase())
			throwException("Exception: DifferentialRobotComp::Worker::stopBase:: Failure comunication exception");
	}
	else
	{
		mutex->unlock();
		throwException("Exception: DifferentialRobotComp::Worker::stopBase:: Worker is sttoped by setting");
	}
}
void DifferentialRobotI::resetOdometer( const Ice::Current&) 
{
	mutex->lock();
	if(worker->active)
	{
		mutex->unlock();
		if(!worker->resetOdometer())
			throwException("Exception: DifferentialRobotComp::Worker::resetOdometer:: Failure comunication exception");
	}
	else
		throwException("Exception: DifferentialRobotComp::Worker::resetOdometer:: Worker is sttoped by setting");
}
void DifferentialRobotI::setOdometer(const RoboCompGenericBase::TBaseState &bState, const Ice::Current&) 
{
	mutex->lock();
	if(worker->active)
	{
		mutex->unlock();
		if(!worker->setOdometer(bState))
			throwException("Exception: DifferentialRobotComp::Worker::setOdometer:: Failure comunication exception");
	}
	else
		throwException("Exception: DifferentialRobotComp::Worker::getBaseState:: Worker is sttoped by setting");
}
void DifferentialRobotI::setOdometerPose(int x, int z, float alpha, const Ice::Current&) 
{ 
	mutex->lock();
	if(worker->active)
	{
		mutex->unlock();
		RoboCompGenericBase::TBaseState bState;
		bState.correctedX     = bState.x     = x;
		bState.correctedZ     = bState.z     = z;
		bState.correctedAlpha = bState.alpha = alpha;
		if (!worker->setOdometer(bState))
			throwException("Exception: DifferentialRobotComp::Worker::setOdometerPose:: Failure comunication exception");
	}
	else
		throwException("Exception: DifferentialRobotComp::Worker::setOdometerPose:: Worker is sttoped by setting");
}
void DifferentialRobotI::getMechParams(RoboCompDifferentialRobot::TMechParams &mechParams, const Ice::Current &){
	mutex->lock();
	if(worker->active)
	{
		mutex->unlock();
		mechParams = worker->getMechParams();
	}
	else
		throwException("Exception: DifferentialRobotComp::Worker::getMechanicalParams:: Worker is sttoped by setting");
}

void DifferentialRobotI::throwException(std::string msg)
{
	RoboCompGenericBase::HardwareFailedException ex;
	ex.what = msg;
	throw ex;
}


void DifferentialRobotI::correctOdometer(int x, int z, float alpha, const Ice::Current&)
{
	if (worker->active)
	{
		worker->correctOdometer(x, z, alpha);
	}
	else
	{
		throwException("Exception: DifferentialRobotComp::Worker::setOdometerPose:: Worker is sttoped by setting");
	}
}

