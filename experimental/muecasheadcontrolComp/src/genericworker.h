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
#ifndef GENERICWORKER_H
#define GENERICWORKER_H

// #include <ipp.h>
#include <QtGui>
#include <stdint.h>
#include <qlog/qlog.h>
#include <CommonBehavior.h>
#include <ui_guiDlg.h>
#include <Camera.h>
#include <JointMotor.h>
#include <IMU.h>
#include <CommonHead.h>
#include <muecasheadcontrol.h>
#include <JoystickAdapter.h>
#include "rcdraw/rcdraw.h"
#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100

typedef map <string,::IceProxy::Ice::Object*> MapPrx;

using namespace std;

/**
       \brief
       @author authorname
*/
using namespace RoboCompCommonHead;
using namespace RoboCompmuecasheadcontrol;
using namespace RoboCompJoystickAdapter;

class GenericWorker : public QWidget, public Ui_guiDlg
{
Q_OBJECT
public:
	GenericWorker(MapPrx& mprx);
	virtual ~GenericWorker();
	virtual void killYourSelf();
	virtual void setPeriod(int p);
	
	virtual void setParams(RoboCompCommonBehavior::ParameterList params) = 0;
	QMutex *mutex;                //Shared mutex with servant

	RoboCompCamera::CameraPrx camera_proxy;
	RoboCompJointMotor::JointMotorPrx jointmotor0_proxy;
	RoboCompJointMotor::JointMotorPrx jointmotor1_proxy;
	RoboCompIMU::IMUPrx imu_proxy;
	virtual void resetHead() = 0;
	virtual void stopHead() = 0;
	virtual void setPanLeft(float pan) = 0;
	virtual void setPanRight(float pan) = 0;
	virtual void setTilt(float tilt) = 0;
	virtual void setNeck(float neck) = 0;
	virtual void saccadic2DLeft(float leftPan, float tilt) = 0;
	virtual void saccadic2DRight(float rightPan, float tilt) = 0;
	virtual void saccadic3D(float leftPan, float rightPan, float tilt) = 0;
	virtual void saccadic4D(float leftPan, float rightPan, float tilt, float neck) = 0;
	virtual void setNMotorsPosition(const RoboCompJointMotor::MotorGoalPositionList& listGoals) = 0;
	virtual RoboCompCommonHead::THeadParams getHeadParams() = 0;
	virtual void getHeadState(RoboCompCommonHead::THeadState& hState) = 0;
	virtual bool isMovingHead() = 0;

	virtual RoboCompJointMotor::MotorParamsList getAllMotorParams() = 0;
	virtual void  getAllMotorState(RoboCompJointMotor::MotorStateMap& mstateMap) = 0;
	virtual void  setPosition(const RoboCompJointMotor::MotorGoalPosition& goal) = 0;

	virtual void  sendData(const TData& data) = 0;

protected:
	QTimer timer;
	int Period;
private:
	MapPrx map;
public slots:
	virtual void compute() = 0;
signals:
	void kill();
};

#endif