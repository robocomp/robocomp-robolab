/*
 *    Copyright (C) 2009-2010 by RoboLab - University of Extremadura
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
#ifndef JOINTMOTORI_H
#define JOINTMOTORI_H

#include <QtCore>

#include <Ice/Ice.h>
#include <JointMotor.h>

#include "worker.h"

using namespace RoboCompJointMotor;

class JointMotorI : public virtual RoboCompJointMotor::JointMotor
{
public:
	JointMotorI(Worker *_worker);
	~JointMotorI();

	QMutex *mutex;
	
	void setPosition(const MotorGoalPosition & goalPos, const Ice::Current& = ::Ice::Current());
	void setVelocity(const MotorGoalVelocity & goalVel, const Ice::Current& = ::Ice::Current());
	void setZeroPos(const std::string& name, const Ice::Current& = ::Ice::Current());
	void setSyncZeroPos(const Ice::Current& = ::Ice::Current());
	void setSyncPosition(const MotorGoalPositionList & goalPosList, const Ice::Current & = ::Ice::Current());    
	void setSyncVelocity(const MotorGoalVelocityList & goalVelList, const Ice::Current & = ::Ice::Current());    
	MotorParams getMotorParams( const ::std::string & motor , const Ice::Current& = ::Ice::Current());
	BusParams getBusParams(const Ice::Current & = ::Ice::Current());
	MotorState getMotorState(const ::std::string & motor , const Ice::Current& = ::Ice::Current());
	MotorStateMap getMotorStateMap(const MotorList & motorList, const ::Ice::Current& = ::Ice::Current());
	MotorParamsList getAllMotorParams(const ::Ice::Current& = ::Ice::Current());
	void getAllMotorState(MotorStateMap & mstateMap, const ::Ice::Current& = ::Ice::Current());

	void stopAllMotors(const Ice::Current& = ::Ice::Current());
	void stopMotor(const ::std::string &motor, const Ice::Current& = ::Ice::Current());
	void releaseBrakeAllMotors(const Ice::Current& = ::Ice::Current());
	void releaseBrakeMotor(const ::std::string &motor, const Ice::Current& = ::Ice::Current());
	void enableBrakeAllMotors(const Ice::Current& = ::Ice::Current());
	void enableBrakeMotor(const ::std::string &motor, const Ice::Current& = ::Ice::Current());

private:
	Worker *worker;
};

#endif
