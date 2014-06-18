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
#include "dynamixel_serial.h"
#include "dynamixel_sdk.h"
#include "handler.h"
#include <qlog/qlog.h>
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

	QMutex *w_mutex;                //Shared mutex with servant
	
//JointMotor
	void getState(const QString & motor, RoboCompJointMotor::MotorState & state);
	void setPosition( const RoboCompJointMotor::MotorGoalPosition & goalPosition);
	void setVelocity( const RoboCompJointMotor::MotorGoalVelocity & goalVelocity);
	void setSyncVelocity( const RoboCompJointMotor::MotorGoalVelocityList & goalVelList);
	void setReferenceVelocity( const RoboCompJointMotor::MotorGoalVelocity & goalVelocity);
	void setSyncPosition( const RoboCompJointMotor::MotorGoalPositionList & goalPosList );
	void getMotorParams( const QString & motor, RoboCompJointMotor::MotorParams & mp);
	RoboCompJointMotor::MotorParamsList getAllMotorParams();
	RoboCompJointMotor::MotorStateMap getAllMotorState();
	RoboCompJointMotor::BusParams getBusParams();
	void setZeroPos(const std::string &motor);
	void setSyncZeroPos();
	void stopAllMotors();
	void stopMotor(const QString &motor);
	void releaseBrakeAllMotors();
	void releaseBrakeMotor(const QString &motor);
	void enableBrakeAllMotors();
	void enableBrakeMotor(const QString &motor);
//CommonBehavior
	void setPeriod(int period);
	int getPeriod();
	bool setParams(RoboCompCommonBehavior::ParameterList _params);
	void killYourSelf();
	
  private:
	void run();

	
  private:
	QTimer timer;
	Handler *handler;
	bool active;
	int period;
	QMutex *monitor_mutex; // Control access for monitor-related resources
	RoboCompJointMotor::MotorParamsList params;
	RoboCompJointMotor::BusParams busParams;
	
	RoboCompJointMotor::HardwareFailedException hFailed;
	
  public slots:

  signals:
	void kill();

};

#endif
