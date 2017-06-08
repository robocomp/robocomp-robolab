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
#include "genericbaseI.h"

GenericBaseI::GenericBaseI(Worker *_worker, QObject *parent) : QObject(parent)
{
	worker = _worker;
	mutex = worker->mutex;       // Shared worker mutex
	// Component initialization...
}


GenericBaseI::~GenericBaseI()
{
	// Free component resources here
}

// Component functions, implementation
void GenericBaseI::getBaseState(RoboCompGenericBase::TBaseState& state, const Ice::Current&)
{
	mutex->lock();
	if(worker->active)
	{
		mutex->unlock();
		state = worker->getBaseState();
	}
	else
	{
		mutex->unlock();
		throwException("Exception: DifferentialRobotComp::Worker::getBaseState:: worker is sttoped by setting");
	}
}

void GenericBaseI::getBasePose(Ice::Int& x, Ice::Int& z, Ice::Float& alpha, const Ice::Current&)
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
void GenericBaseI::throwException(std::string msg)
{
	RoboCompGenericBase::HardwareFailedException ex;
	ex.what = msg;
	throw ex;
}

