/*
 *    Copyright (C) 2010 by RoboLab - University of Extremadura
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
#include "jointmotorI.h"

JointMotorI::JointMotorI( Worker *_worker)
{
	worker = _worker;
	mutex = worker->w_mutex;
}


JointMotorI::~JointMotorI()
{
	// Free component resources here
}

// Component functions, implementation

void JointMotorI::setPosition( const MotorGoalPosition & goalPos, const Ice::Current&)
{
	worker->setPosition( goalPos );
}

void JointMotorI::setSyncPosition( const MotorGoalPositionList & goalPosList, const Ice::Current &)        
{
	worker->setSyncPosition( goalPosList);
}

void JointMotorI::setSyncVelocity(const MotorGoalVelocityList & goalVelList, const Ice::Current &)
{
	worker->setSyncVelocity( goalVelList);
}

void JointMotorI::setVelocity( const MotorGoalVelocity & goalVel, const Ice::Current&)
{
	worker->setReferenceVelocity(goalVel);
}

MotorParams JointMotorI::getMotorParams( const ::std::string& motor , const Ice::Current &)
{
	MotorParams mp;
	worker->getMotorParams(QString::fromStdString(motor), mp);
	return mp;
}

BusParams JointMotorI::getBusParams( const Ice::Current & )
{
	return worker->getBusParams();
}

MotorState JointMotorI::getMotorState( const ::std::string & motor , const Ice::Current &)
{
	MotorState state;
	worker->getState(QString::fromStdString(motor), state);
	return state;
}

MotorStateMap JointMotorI::getMotorStateMap( const MotorList &motorList, const ::Ice::Current&)
{
	MotorStateMap stateMap;
	MotorState state;
	for (auto motor: motorList)
	{
			worker->getState( QString::fromStdString( motor ) , state);
			stateMap[motor] = state ;
	}
	return stateMap;
}

MotorParamsList JointMotorI::getAllMotorParams( const ::Ice::Current& )
{
	MotorParamsList list = worker->getAllMotorParams();

	if (list.empty() == false)
		return list;
	else
		throw RoboCompJointMotor::UnknownMotorException("JointMotor::getAllMotorParams - Empty List");
}

void JointMotorI::getAllMotorState(MotorStateMap &mstateMap, const ::Ice::Current&)
{
	mstateMap = worker->getAllMotorState();
}

void JointMotorI::setZeroPos(const std::string &name, const Ice::Current&)
{
	worker->setZeroPos(name);
}

void JointMotorI::setSyncZeroPos(const Ice::Current&)
{
	worker->setSyncZeroPos();
}

void JointMotorI::stopAllMotors(const Ice::Current&)
{
	throw RoboCompJointMotor::HardwareFailedException("Not implemented yet");
}

void JointMotorI::stopMotor(const ::std::string &motor, const Ice::Current& )
{
	RoboCompJointMotor::MotorGoalVelocity goalVel;
	goalVel.velocity = 0.;
	goalVel.maxAcc = 10.;
	goalVel.name = motor;
	worker->setReferenceVelocity(goalVel);
}

void JointMotorI::releaseBrakeAllMotors(const Ice::Current& )
{
	throw RoboCompJointMotor::HardwareFailedException("Not implemented yet");
}

void JointMotorI::releaseBrakeMotor(const ::std::string &motor, const Ice::Current& )
{
	throw RoboCompJointMotor::HardwareFailedException("Not implemented yet");
}

void JointMotorI::enableBrakeAllMotors(const Ice::Current& )
{
	throw RoboCompJointMotor::HardwareFailedException("Not implemented yet");
}

void JointMotorI::enableBrakeMotor(const ::std::string &motor, const Ice::Current& )
{
	throw RoboCompJointMotor::HardwareFailedException("Not implemented yet");
}

