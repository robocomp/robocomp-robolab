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
#ifndef WORKER_H
#define WORKER_H

// #include <ipp.h>
#include <QtCore>
#include "dunkermotoren.h"
#include <JointMotor.h>
#include <CommonBehavior.h>
#include <servo.h>

#define CHECK_PERIOD 5000
#define BASIC_PERIOD 50

/**
       \brief
       @author Robolab
*/


class Worker : public QThread
{
Q_OBJECT
public:
	Worker(QObject *parent = 0);
	~Worker();
	QMutex *monitor_mutex; // Control access to monitor-related resources
	QMutex *w_mutex;       // Control access to Worker resources
	RoboCompJointMotor::MotorParamsList params;
	RoboCompJointMotor::BusParams busParams;
	QHash<int,DunkerParams> dunkerParams;
	QVector<QString> motor_names;

	// State
	void getState(const QString & motor, RoboCompJointMotor::MotorState & state);
	RoboCompJointMotor::BusParams getBusParams();
	void getMotorParams( const QString & motor, RoboCompJointMotor::MotorParams & mp);
	RoboCompJointMotor::MotorParamsList getAllMotorParams();
	RoboCompJointMotor::MotorStateMap getAllMotorState();

	// Velocity
	void setVelocity(const RoboCompJointMotor::MotorGoalVelocity & goalVelocity);
	void setReferenceVelocity(const RoboCompJointMotor::MotorGoalVelocity & goalVelocity);
	void setSyncVelocity(const RoboCompJointMotor::MotorGoalVelocityList & goalVelList);

	// Position
	void setPosition(const RoboCompJointMotor::MotorGoalPosition & goalPosition);
	void setSyncPosition(const RoboCompJointMotor::MotorGoalPositionList & goalPosList);
	void setZeroPos(const std::string &name);
	void setSyncZeroPos();

	// CommonBehavior
	void setParams();
	void setFrequency(int freq);
	void setParams(RoboCompCommonBehavior::ParameterList _params);
	int getFrequency();
	void run();

private:
	QTimer timer;

	Dunkermotoren *handler;
	bool active;
	int frequency;

};

#endif
