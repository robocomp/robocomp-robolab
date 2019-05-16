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
#ifndef GENERICWORKER_H
#define GENERICWORKER_H

#include "config.h"
#include <stdint.h>
#include <qlog/qlog.h>


#include <CommonBehavior.h>

#include <JointMotor.h>
#include <GenericBase.h>
#include <HumanTracker.h>
#include <RGBD.h>
#include <HumanTrackerJointsAndRGB.h>

#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100

using namespace std;
using namespace RoboCompJointMotor;
using namespace RoboCompGenericBase;
using namespace RoboCompHumanTracker;
using namespace RoboCompRGBD;

typedef map <string,::IceProxy::Ice::Object*> MapPrx;


class GenericWorker :
public QObject
{
Q_OBJECT
public:
	GenericWorker(MapPrx& mprx);
	virtual ~GenericWorker();
	virtual void killYourSelf();
	virtual void setPeriod(int p);

	virtual bool setParams(RoboCompCommonBehavior::ParameterList params) = 0;
	QMutex *mutex;


	RoboCompHumanTrackerJointsAndRGB::HumanTrackerJointsAndRGBPrx humantrackerjointsandrgb_pubproxy;

	virtual void HumanTracker_getJointsPosition(const int id, jointListType &jointList) = 0;
	virtual void HumanTracker_getRTMatrixList(const int id, RTMatrixList &RTMatList) = 0;
	virtual void HumanTracker_getUser(const int id, TPerson &user) = 0;
	virtual bool HumanTracker_getJointDepthPosition(const int idperson, const string &idjoint, joint &depthjoint) = 0;
	virtual void HumanTracker_getUsersList(PersonList &users) = 0;
	virtual void HumanTracker_getUserState(const int id, TrackingState &state) = 0;
	virtual Registration RGBD_getRegistration() = 0;
	virtual void RGBD_getData(imgType &rgbMatrix, depthType &distanceMatrix, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState &bState) = 0;
	virtual void RGBD_getXYZ(PointSeq &points, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState &bState) = 0;
	virtual void RGBD_getRGB(ColorSeq &color, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState &bState) = 0;
	virtual TRGBDParams RGBD_getRGBDParams() = 0;
	virtual void RGBD_getDepth(DepthSeq &depth, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState &bState) = 0;
	virtual void RGBD_setRegistration(const Registration &value) = 0;
	virtual void RGBD_getXYZByteStream(imgType &pointStream, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState &bState) = 0;
	virtual void RGBD_getImage(ColorSeq &color, DepthSeq &depth, PointSeq &points, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState &bState) = 0;
	virtual void RGBD_getDepthInIR(depthType &distanceMatrix, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState &bState) = 0;

protected:
	QTimer timer;
	int Period;

private:


public slots:
	virtual void compute() = 0;
	virtual void initialize(int period) = 0;
signals:
	void kill();
};

#endif
