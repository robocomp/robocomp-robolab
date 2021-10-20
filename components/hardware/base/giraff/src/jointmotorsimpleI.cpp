/*
 *    Copyright (C) 2021 by YOUR NAME HERE
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
#include "jointmotorsimpleI.h"

JointMotorSimpleI::JointMotorSimpleI(GenericWorker *_worker)
{
	worker = _worker;
}


JointMotorSimpleI::~JointMotorSimpleI()
{
}


RoboCompJointMotorSimple::MotorParams JointMotorSimpleI::getMotorParams(std::string motor, const Ice::Current&)
{
	return worker->JointMotorSimple_getMotorParams(motor);
}

RoboCompJointMotorSimple::MotorState JointMotorSimpleI::getMotorState(std::string motor, const Ice::Current&)
{
	return worker->JointMotorSimple_getMotorState(motor);
}

void JointMotorSimpleI::setPosition(std::string name, RoboCompJointMotorSimple::MotorGoalPosition goal, const Ice::Current&)
{
	worker->JointMotorSimple_setPosition(name, goal);
}

void JointMotorSimpleI::setVelocity(std::string name, RoboCompJointMotorSimple::MotorGoalVelocity goal, const Ice::Current&)
{
	worker->JointMotorSimple_setVelocity(name, goal);
}

void JointMotorSimpleI::setZeroPos(std::string name, const Ice::Current&)
{
	worker->JointMotorSimple_setZeroPos(name);
}

