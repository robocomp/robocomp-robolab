/*
 *    Copyright (C) 2017 by YOUR NAME HERE
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
#include <Camera.h>
#include <DifferentialRobot.h>
#include <GetAprilTags.h>
#include <CommonHead.h>
#include <RGBD.h>
#include <RGBDBus.h>
#include <JointMotor.h>
#include <AprilTags.h>



#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100

typedef map <string,::IceProxy::Ice::Object*> MapPrx;

using namespace std;

using namespace RoboCompCamera;
using namespace RoboCompDifferentialRobot;
using namespace RoboCompGetAprilTags;
using namespace RoboCompCommonHead;
using namespace RoboCompRGBD;
using namespace RoboCompRGBDBus;
using namespace RoboCompJointMotor;
using namespace RoboCompAprilTags;




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
	

	AprilTagsPrx apriltags_proxy;
	RGBDBusPrx rgbdbus_proxy;
	CameraPrx camera_proxy;
	RGBDPrx rgbd_proxy;

	virtual listaMarcas checkMarcas() = 0;


protected:
	QTimer timer;
	int Period;

public slots:
	virtual void compute() = 0;
signals:
	void kill();
};

#endif
