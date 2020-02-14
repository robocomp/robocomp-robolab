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
#include <QtCore>
#include <stdint.h>
#include <DifferentialRobot.h>
#include <CommonBehavior.h>
#include <GenericBase.h>
#include "handler.h"
#include "gazebohandler.h"
#include "playerhandler.h"
#include <qlog/qlog.h>

/**
       \brief
       @author authorname
*/
class Worker : public QThread
{
Q_OBJECT
  public:
	Worker();
	~Worker();
	QMutex *mutex;                //Shared mutex with servant
	void run();

	//DifferentialRobot
	bool setSpeedBase(float adv ,float rot  );
	bool stopBase();
	RoboCompGenericBase::TBaseState getBaseState();
	RoboCompDifferentialRobot::TMechParams getMechParams();
	bool resetOdometer();
	bool setOdometer(RoboCompGenericBase::TBaseState bState);
	void correctOdometer(float x, float z, float alpha);
	//CommonBehavior
	void setPeriod(int period);
	int getPeriod();
	void setParams(RoboCompCommonBehavior::ParameterList _params);
  private:
	Handler *handler;
	QTimer timer;
	int period;
  public: 
	bool active;
	RoboCompDifferentialRobot::TMechParams params;
  signals:
	void kill();

};

#endif
