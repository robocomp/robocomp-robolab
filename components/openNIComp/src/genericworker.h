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
#include <RGBD.h>
#include <HumanTracker.h>

#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100

typedef map <string,::IceProxy::Ice::Object*> MapPrx;

using namespace std;

/**
       \brief
       @author authorname
*/
using namespace RoboCompRGBD;
using namespace RoboCompHumanTracker;

class GenericWorker : public QObject
{
Q_OBJECT
public:
	GenericWorker(MapPrx& mprx, QObject *parent = 0);
	virtual ~GenericWorker();
	virtual void killYourSelf();
	virtual void setPeriod(int p);
	
	virtual void setParams(RoboCompCommonBehavior::ParameterList params) = 0;
	QMutex *mutex;                //Shared mutex with servant

	virtual TRGBDParams getRGBDParams() = 0;
virtual void  setRegistration(const Registration& value) = 0;
virtual Registration getRegistration() = 0;
virtual void  getData(imgType& rgbMatrix, depthType& distanceMatrix, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState) = 0;
virtual void  getDepthInIR(depthType& distanceMatrix, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState) = 0;
virtual void  getImage(ColorSeq& color, DepthSeq& depth, PointSeq& points, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState) = 0;
virtual void  getDepth(DepthSeq& depth, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState) = 0;
virtual void  getRGB(ColorSeq& color, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState) = 0;
virtual void  getXYZ(PointSeq& points, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState) = 0;

virtual trackingState getState() = 0;
virtual void  getRTMatrixList(RTMatrixList& RTMatList) = 0;
virtual void  getJointsPosition(jointListType& jointList) = 0;
virtual void  getData(RTMatrixList& RTMatList, jointListType& jointList, trackingState& state) = 0;

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