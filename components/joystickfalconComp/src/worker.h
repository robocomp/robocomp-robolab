/*
 *    Copyright (C) 2006-2011 by RoboLab - University of Extremadura
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
#include <CommonBehavior.h>
#include <JoystickAdapter.h>

#include "falconhandler.h"


#define BASIC_PERIOD 100

using namespace std;

/**
       \brief
       @author authorname
*/
class Worker  : public QThread
{
Q_OBJECT
public:
	Worker( unsigned int numDev, RoboCompJoystickAdapter::JoystickAdapterPrx joystickAdapterPrx, RoboCompJoystickAdapter::JoystickAdapterPrx joystickAdapterPrx2 );
	~Worker();
	//CommonBehavior
	void killYourSelf();
	void setPeriod(int p);
	bool setParams(RoboCompCommonBehavior::ParameterList params_);
	void run(); //Thread
	QMutex *mutex;                //Shared mutex with servant

private:
	int Period;
	RoboCompJoystickAdapter::JoystickAdapterPrx joystickAdapter,joystickAdapter2;
	FalconHandler falcon;

signals:
	void kill();
};

#endif
