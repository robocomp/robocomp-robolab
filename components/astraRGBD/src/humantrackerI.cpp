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
#include "humantrackerI.h"

HumanTrackerI::HumanTrackerI(GenericWorker *_worker)
{
	worker = _worker;
}


HumanTrackerI::~HumanTrackerI()
{
}

void HumanTrackerI::getJointsPosition(const int  id,  jointListType  &jointList, const Ice::Current&)
{
	worker->HumanTracker_getJointsPosition(id, jointList);
}

void HumanTrackerI::getRTMatrixList(const int  id,  RTMatrixList  &RTMatList, const Ice::Current&)
{
	worker->HumanTracker_getRTMatrixList(id, RTMatList);
}

void HumanTrackerI::getUser(const int  id,  TPerson  &user, const Ice::Current&)
{
	worker->HumanTracker_getUser(id, user);
}

bool HumanTrackerI::getJointDepthPosition(const int  idperson, const string  &idjoint,  joint  &depthjoint, const Ice::Current&)
{
	return worker->HumanTracker_getJointDepthPosition(idperson, idjoint, depthjoint);
}

void HumanTrackerI::getUsersList( PersonList  &users, const Ice::Current&)
{
	worker->HumanTracker_getUsersList(users);
}

void HumanTrackerI::getUserState(const int  id,  TrackingState  &state, const Ice::Current&)
{
	worker->HumanTracker_getUserState(id, state);
}

