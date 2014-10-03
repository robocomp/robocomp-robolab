/*
 *    Copyright (C) 2006-2011 by RoboLab - University of Extremadura
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
#include "laserI.h"
/**
* \brief Default constructor
*/
LaserI::LaserI(Worker *_worker, QObject *parent) : QObject(parent)
{
	worker = _worker;
	mutex = worker->mutex;       // Shared worker mutex
	// Component initialization...
}

/**
* \brief Default destructor
*/
LaserI::~LaserI()
{
	// Free component resources here
}

// Component functions, implementation

TLaserData LaserI::getLaserData( const Ice::Current&) 
{ 
	return worker->getLaserData();
}

TLaserData LaserI::getLaserAndBStateData(RoboCompDifferentialRobot::TBaseState&bState, const Ice::Current&) 
{ 
	return worker->getLaserAndBStateData(bState);
}

LaserConfData LaserI::getLaserConfData( const Ice::Current&) 
{ 
	return worker->getLaserConfData();
}


