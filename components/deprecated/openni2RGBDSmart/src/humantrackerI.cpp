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
#include "humantrackerI.h"

HumanTrackerI::HumanTrackerI(GenericWorker *_worker, QObject *parent) : QObject(parent)
{
	worker = _worker;
	mutex = worker->mutex;       // Shared worker mutex
	// Component initialization...
}


HumanTrackerI::~HumanTrackerI()
{
	// Free component resources here
}

// Component functions, implementation
void HumanTrackerI::getJointsPosition(Ice::Int id, jointListType& jointList, const Ice::Current&){
	worker->getJointsPosition(id,jointList);
}

void HumanTrackerI::getRTMatrixList(Ice::Int id, RTMatrixList& RTMatList, const Ice::Current&){
	worker->getRTMatrixList(id,RTMatList);
}

void HumanTrackerI::getUserState(Ice::Int id, TrackingState& state, const Ice::Current&){
	worker->getUserState(id,state);
}

void HumanTrackerI::getUser(Ice::Int id, TPerson& user, const Ice::Current&){
	worker->getUser(id,user);
}

void HumanTrackerI::getUsersList(PersonList& users, const Ice::Current&){
	worker->getUsersList(users);
}


