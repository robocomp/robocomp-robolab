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
#ifndef COMMONHEADI_H
#define COMMONHEADI_H

// QT includes
#include <QtCore/QObject>

// Ice includes
#include <Ice/Ice.h>
#include <CommonHead.h>

#include <config.h>
#include "genericworker.h"

using namespace RoboCompCommonHead;

class CommonHeadI : public QObject , public virtual RoboCompCommonHead::CommonHead
{
Q_OBJECT
public:
	CommonHeadI( GenericWorker *_worker, QObject *parent = 0 );
	~CommonHeadI();
	void resetHead(const Ice::Current& = Ice::Current());
	void stopHead(const Ice::Current& = Ice::Current());
	void setPanLeft(float pan, const Ice::Current& = Ice::Current());
	void setPanRight(float pan, const Ice::Current& = Ice::Current());
	void setTilt(float tilt, const Ice::Current& = Ice::Current());
	void setNeck(float neck, const Ice::Current& = Ice::Current());
	void saccadic2DLeft(float leftPan, float tilt, const Ice::Current& = Ice::Current());
	void saccadic2DRight(float rightPan, float tilt, const Ice::Current& = Ice::Current());
	void saccadic3D(float leftPan, float rightPan, float tilt, const Ice::Current& = Ice::Current());
	void saccadic4D(float leftPan, float rightPan, float tilt, float neck, const Ice::Current& = Ice::Current());
	void setNMotorsPosition(const RoboCompJointMotor::MotorGoalPositionList& listGoals, const Ice::Current& = Ice::Current());
	RoboCompCommonHead::THeadParams getHeadParams(const Ice::Current& = Ice::Current());
	void getHeadState(RoboCompCommonHead::THeadState& hState, const Ice::Current& = Ice::Current());
	bool isMovingHead(const Ice::Current& = Ice::Current());
	

	QMutex *mutex;
private:

	GenericWorker *worker;
public slots:


};

#endif