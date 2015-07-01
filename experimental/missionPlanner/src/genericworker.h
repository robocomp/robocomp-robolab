/*
 *    Copyright (C) 2015 by YOUR NAME HERE
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

#include <ui_mainUI.h>

#include <CommonBehavior.h>
#include <TrajectoryRobot2D.h>
#include <AprilTags.h>
#include <ASRPublish.h>



#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100

typedef map <string,::IceProxy::Ice::Object*> MapPrx;

using namespace std;

using namespace RoboCompTrajectoryRobot2D;
using namespace RoboCompAprilTags;
using namespace RoboCompASRPublish;




class GenericWorker : 
#ifdef USE_QTGUI
public QWidget, public Ui_guiDlg
#else
public QObject
#endif
{
Q_OBJECT
public:
	GenericWorker(MapPrx& mprx);
	virtual ~GenericWorker();
	virtual void killYourSelf();
	virtual void setPeriod(int p);
	
	virtual bool setParams(RoboCompCommonBehavior::ParameterList params) = 0;
	QMutex *mutex;
	

	ASRPublishPrx asrpublish_proxy;
	TrajectoryRobot2DPrx trajectoryrobot2d_proxy;

	virtual void newAprilTag(const tagsList &tags) = 0;


protected:
	QTimer timer;
	int Period;

public slots:
	virtual void compute() = 0;
signals:
	void kill();
};

#endif