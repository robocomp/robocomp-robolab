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
#include "omnirobotI.h"

OmniRobotI::OmniRobotI(GenericWorker *_worker, QObject *parent) : QObject(parent)
{
	worker = _worker;
	mutex = worker->mutex;       // Shared worker mutex
}


OmniRobotI::~OmniRobotI()
{
}

void OmniRobotI::correctOdometer(const int  x, const int  z, const float  alpha, const Ice::Current&)
{
	worker->correctOdometer(x, z, alpha);
}

void OmniRobotI::getBasePose( int  &x,  int  &z,  float  &alpha, const Ice::Current&)
{
	worker->getBasePose(x, z, alpha);
}

void OmniRobotI::resetOdometer(const Ice::Current&)
{
	worker->resetOdometer();
}

void OmniRobotI::setOdometer(const TBaseState  &state, const Ice::Current&)
{
	worker->setOdometer(state);
}

void OmniRobotI::getBaseState( TBaseState  &state, const Ice::Current&)
{
	worker->getBaseState(state);
}

void OmniRobotI::setOdometerPose(const int  x, const int  z, const float  alpha, const Ice::Current&)
{
	worker->setOdometerPose(x, z, alpha);
}

void OmniRobotI::stopBase(const Ice::Current&)
{
	worker->stopBase();
}

void OmniRobotI::setSpeedBase(const float  advx, const float  advz, const float  rot, const Ice::Current&)
{
	worker->setSpeedBase(advx, advz, rot);
}






