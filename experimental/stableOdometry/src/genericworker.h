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
#ifndef GENERICWORKER_H
#define GENERICWORKER_H

#include "config.h"
#include <QtGui>
#include <stdint.h>
#include <qlog/qlog.h>


#include <CommonBehavior.h>

#include <OmniRobot.h>
#include <AprilBasedLocalization.h>
#include <CGR.h>

#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100

typedef map <string,::IceProxy::Ice::Object*> MapPrx;

using namespace std;

using namespace RoboCompOmniRobot;
using namespace RoboCompCGR;
using namespace RoboCompAprilBasedLocalization;




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
	

	CGRPrx cgr_proxy;
	OmniRobotPrx omnirobot_proxy;

	virtual void newAprilBasedPose(const float x, const float z, const float alpha) = 0;
	virtual void newCGRPose(const float poseUncertainty, const float x, const float z, const float alpha) = 0;
	virtual void newCGRCorrection(const float poseUncertainty, const float x1, const float z1, const float alpha1, const float x2, const float z2, const float alpha2) = 0;

protected:
	QTimer timer;
	int Period;

private:


public slots:
	virtual void compute() = 0;
signals:
	void kill();
};

#endif