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
#include "jointmotorI.h"

JointMotorI::JointMotorI(GenericWorker *_worker, QObject *parent) : QObject(parent)
{
	worker = _worker;
	mutex = worker->mutex;       // Shared worker mutex
	// Component initialization...
}


JointMotorI::~JointMotorI()
{
	// Free component resources here
}

// Component functions, implementation
void JointMotorI::setPosition(const MotorGoalPosition& goal, const Ice::Current&){
	worker->setPosition(goal);
}

void JointMotorI::setVelocity(const MotorGoalVelocity& goal, const Ice::Current&){
	worker->setVelocity(goal);
}

void JointMotorI::setZeroPos(const std::string& name, const Ice::Current&){
	worker->setZeroPos(name);
}

void JointMotorI::setSyncPosition(const MotorGoalPositionList& listGoals, const Ice::Current&){
	worker->setSyncPosition(listGoals);
}

void JointMotorI::setSyncVelocity(const MotorGoalVelocityList& listGoals, const Ice::Current&){
	worker->setSyncVelocity(listGoals);
}

void JointMotorI::setSyncZeroPos(const Ice::Current&){
	worker->setSyncZeroPos();
}

MotorParams JointMotorI::getMotorParams(const std::string& motor, const Ice::Current&){
	return worker->getMotorParams(motor);
}

MotorState JointMotorI::getMotorState(const std::string& motor, const Ice::Current&){
	return worker->getMotorState(motor);
}

MotorStateMap JointMotorI::getMotorStateMap(const MotorList& mList, const Ice::Current&){
	return worker->getMotorStateMap(mList);
}

void JointMotorI::getAllMotorState(MotorStateMap& mstateMap, const Ice::Current&){
	worker->getAllMotorState(mstateMap);
}

MotorParamsList JointMotorI::getAllMotorParams(const Ice::Current&){
	return worker->getAllMotorParams();
}

BusParams JointMotorI::getBusParams(const Ice::Current&){
	return worker->getBusParams();
}

void JointMotorI::stopAllMotors(const Ice::Current&)
{
	RoboCompJointMotor::HardwareFailedException hfailed("Not implemented yet");
	throw hfailed;	
}
void JointMotorI::stopMotor(const ::std::string &motor, const Ice::Current& )
{
	RoboCompJointMotor::HardwareFailedException hfailed("Not implemented yet");
	throw hfailed;	
}
void JointMotorI::releaseBrakeAllMotors(const Ice::Current& )
{
	RoboCompJointMotor::HardwareFailedException hfailed("Not implemented yet");
	throw hfailed;	
}
void JointMotorI::releaseBrakeMotor(const ::std::string &motor, const Ice::Current& )
{
	RoboCompJointMotor::HardwareFailedException hfailed("Not implemented yet");
	throw hfailed;	
}
void JointMotorI::enableBrakeAllMotors(const Ice::Current& )
{
	RoboCompJointMotor::HardwareFailedException hfailed("Not implemented yet");
	throw hfailed;	
}
void JointMotorI::enableBrakeMotor(const ::std::string &motor, const Ice::Current& )
{
	RoboCompJointMotor::HardwareFailedException hfailed("Not implemented yet");
	throw hfailed;	
}

	
