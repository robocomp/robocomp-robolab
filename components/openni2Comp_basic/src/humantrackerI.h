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
#ifndef HUMANTRACKERI_H
#define HUMANTRACKERI_H

// QT includes
#include <QtCore/QObject>

// Ice includes
#include <Ice/Ice.h>
#include <HumanTracker.h>

#include <config.h>
#include "genericworker.h"

using namespace RoboCompHumanTracker;

class HumanTrackerI : public QObject , public virtual RoboCompHumanTracker::HumanTracker
{
Q_OBJECT
public:
	HumanTrackerI( GenericWorker *_worker, QObject *parent = 0 );
	~HumanTrackerI();
	void  getJointsPosition(Ice::Int id, jointListType& jointList, const Ice::Current& = Ice::Current());
void  getRTMatrixList(Ice::Int id, RTMatrixList& RTMatList, const Ice::Current& = Ice::Current());
void  getUserState(Ice::Int id, TrackingState& state, const Ice::Current& = Ice::Current());
void  getUser(Ice::Int id, TPerson& user, const Ice::Current& = Ice::Current());
void  getUsersList(PersonList& users, const Ice::Current& = Ice::Current());


	QMutex *mutex;
private:

	GenericWorker *worker;
public slots:


};

#endif