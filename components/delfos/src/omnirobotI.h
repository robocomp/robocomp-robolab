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
#ifndef OMNIROBOT_H
#define OMNIROBOT_H

// QT includes
#include <QtCore/QObject>

// Ice includes
#include <Ice/Ice.h>
#include <OmniRobot.h>

#include <config.h>
#include "genericworker.h"

using namespace RoboCompOmniRobot;

class OmniRobotI : public QObject , public virtual RoboCompOmniRobot::OmniRobot
{
Q_OBJECT
public:
	OmniRobotI( GenericWorker *_worker, QObject *parent = 0 );
	~OmniRobotI();
	
	void correctOdometer(const int  x, const int  z, const float  alpha, const Ice::Current&);
	void getBasePose( int  &x,  int  &z,  float  &alpha, const Ice::Current&);
	void resetOdometer(const Ice::Current&);
	void setOdometer(const TBaseState  &state, const Ice::Current&);
	void getBaseState( TBaseState  &state, const Ice::Current&);
	void setOdometerPose(const int  x, const int  z, const float  alpha, const Ice::Current&);
	void stopBase(const Ice::Current&);
	void setSpeedBase(const float  advx, const float  advz, const float  rot, const Ice::Current&);

	QMutex *mutex;
private:

	GenericWorker *worker;
public slots:


};

#endif
