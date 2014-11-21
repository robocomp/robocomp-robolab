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
#include "config.h"
#include <QtGui>
#include <stdint.h>
#include <qlog/qlog.h>
#include <CommonBehavior.h>
#include <ui_guiDlg.h>
#include "config.h"
#include <InnerModelManager.h>
#include <HumanTracker.h>
#include <RGBD.h>
#include "rcdraw/rcdraw.h"

#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100

typedef map <string,::IceProxy::Ice::Object*> MapPrx;

using namespace std;

/**
       \brief
       @author authorname
*/
using namespace RoboCompInnerModelManager;
using namespace RoboCompHumanTracker;
using namespace RoboCompRGBD;
class GenericWorker :
#ifdef USE_QTGUI
public QWidget, public Ui_guiDlg
#else
public QObject
#endif
{
Q_OBJECT
public:
	GenericWorker(MapPrx& mprx, QObject *parent = 0);
	virtual ~GenericWorker();
	virtual void killYourSelf();
	virtual void setPeriod(int p);
	
	virtual bool setParams(RoboCompCommonBehavior::ParameterList params) = 0;
	QMutex *mutex;                //Shared mutex with servant

	InnerModelManagerPrx innermodelmanager_proxy;
	
	virtual TRGBDParams getRGBDParams() = 0;
	virtual void setRegistration(Registration value) = 0;
	virtual Registration getRegistration() = 0;
	virtual void getData(imgType& rgbMatrix, depthType& distanceMatrix, RoboCompJointMotor::MotorStateMap& hState, RoboCompDifferentialRobot::TBaseState& bState) = 0;
	virtual void getDepthInIR(depthType& distanceMatrix, RoboCompJointMotor::MotorStateMap& hState, RoboCompDifferentialRobot::TBaseState& bState) = 0;
	virtual void getImage(ColorSeq& color, DepthSeq& depth, PointSeq& points, RoboCompJointMotor::MotorStateMap& hState, RoboCompDifferentialRobot::TBaseState& bState) = 0;
	virtual void getDepth(DepthSeq& depth, RoboCompJointMotor::MotorStateMap& hState, RoboCompDifferentialRobot::TBaseState& bState) = 0;
	virtual void getRGB(ColorSeq& color, RoboCompJointMotor::MotorStateMap& hState, RoboCompDifferentialRobot::TBaseState& bState) = 0;
	virtual void getXYZ(PointSeq& points, RoboCompJointMotor::MotorStateMap& hState, RoboCompDifferentialRobot::TBaseState& bState) = 0;

	virtual void  getJointsPosition(int id, jointListType& jointList) = 0;
	virtual void  getRTMatrixList(int id, RTMatrixList& RTMatList) = 0;
	virtual void  getUserState(int id, TrackingState& state) = 0;
	virtual void  getUser(int id, TPerson& user) = 0;
	virtual void  getUsersList(PersonList& users) = 0;
protected:
	QTimer timer;
	int Period;
public slots:
	virtual void compute() = 0;
signals:
	void kill();
};

#endif