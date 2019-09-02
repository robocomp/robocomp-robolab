/*
 *    Copyright (C) 2019 by YOUR NAME HERE
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

DifferentialRobotI::DifferentialRobotI(GenericWorker *_worker)
{
	worker = _worker;
}


DifferentialRobotI::~DifferentialRobotI()
{
}

void DifferentialRobotI::correctOdometer(const int  x, const int  z, const float  alpha, const Ice::Current&)
{
	worker->DifferentialRobot_correctOdometer(x, z, alpha);
}

void DifferentialRobotI::getBasePose( int  &x,  int  &z,  float  &alpha, const Ice::Current&)
{
	worker->DifferentialRobot_getBasePose(x, z, alpha);
}

void DifferentialRobotI::resetOdometer(const Ice::Current&)
{
	worker->DifferentialRobot_resetOdometer();
}

void DifferentialRobotI::setOdometer(const RoboCompGenericBase::TBaseState  &state, const Ice::Current&)
{
	worker->DifferentialRobot_setOdometer(state);
}

void DifferentialRobotI::getBaseState( RoboCompGenericBase::TBaseState  &state, const Ice::Current&)
{
	worker->DifferentialRobot_getBaseState(state);
}

void DifferentialRobotI::setOdometerPose(const int  x, const int  z, const float  alpha, const Ice::Current&)
{
	worker->DifferentialRobot_setOdometerPose(x, z, alpha);
}

void DifferentialRobotI::stopBase(const Ice::Current&)
{
	worker->DifferentialRobot_stopBase();
}

void DifferentialRobotI::setSpeedBase(const float  adv, const float  rot, const Ice::Current&)
{
	worker->DifferentialRobot_setSpeedBase(adv, rot);
}

