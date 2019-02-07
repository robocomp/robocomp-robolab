/*
 *    Copyright (C)2019 by YOUR NAME HERE
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
#ifndef HUMANTRACKER_H
#define HUMANTRACKER_H

// Ice includes
#include <Ice/Ice.h>
#include <HumanTracker.h>

#include <config.h>
#include "genericworker.h"

using namespace RoboCompHumanTracker;

class HumanTrackerI : public virtual RoboCompHumanTracker::HumanTracker
{
public:
HumanTrackerI(GenericWorker *_worker);
	~HumanTrackerI();

	void getJointsPosition(const int  id,  jointListType  &jointList, const Ice::Current&);
	void getRTMatrixList(const int  id,  RTMatrixList  &RTMatList, const Ice::Current&);
	void getUser(const int  id,  TPerson  &user, const Ice::Current&);
	bool getJointDepthPosition(const int  idperson, const string  &idjoint,  joint  &depthjoint, const Ice::Current&);
	void getUsersList( PersonList  &users, const Ice::Current&);
	void getUserState(const int  id,  TrackingState  &state, const Ice::Current&);

private:

	GenericWorker *worker;

};

#endif
