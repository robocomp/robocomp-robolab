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
#ifndef WORKER_H
#define WORKER_H

// #include <ipp.h>
#include <QtGui>
#include <stdint.h>
#include <qlog/qlog.h>
#include <q4serialport/q4serialport.h>

#include <CommonBehavior.h>
#include <DifferentialRobot.h>
#include <generichandler.h>
#include <hokuyogenerichandler.h>
#include <hokuyohandler.h>
#include <Laser.h>

#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100


/**
       \brief
       @author authorname
*/
class Worker : public QObject
{
Q_OBJECT
private:
	int Period;
	RoboCompDifferentialRobot::DifferentialRobotPrx differentialrobot;
	RoboCompLaser::LaserConfData laserConf;
	GenericLaserHandler *lh;
public:
	QMutex *mutex;                //Shared mutex with servant
  
	Worker(RoboCompDifferentialRobot::DifferentialRobotPrx differentialrobotprx, QObject *parent = 0);
	~Worker();

	RoboCompLaser::TLaserData getNewData();
	RoboCompLaser::LaserConfData getLaserConf();
	RoboCompDifferentialRobot::TBaseState getBaseState();
	
	//CommonBehavior
	void setPeriod(int period);
	int getPeriod();
	void setParams(RoboCompCommonBehavior::ParameterList _params);
	void killYourSelf();
	
signals:
	void kill();
};

#endif
