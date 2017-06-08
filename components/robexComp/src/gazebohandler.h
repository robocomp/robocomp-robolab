/*
 *    Copyright (C) 2010 by RoboLab - University of Extremadura
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
#if COMPILE_GAZEBO==1

#ifndef GAZEBOBASEHANDLER_H
#define GAZEBOBASEHANDLER_H

#define OLD_API

#if defined(signals) && defined(QOBJECTDEFS_H) && !defined(QT_MOC_CPP)
#  undef signals
#  define signals signals
#endif

#ifdef OLD_API
#include <gazebo/gazebo.h>
#else
#include <gazebo/gz.h>

#endif
// #include <gazebo/GazeboError.hh>
namespace boost
{
  namespace signalslib = signals;
}
#if defined(signals) && defined(QOBJECTDEFS_H) && !defined(QT_MOC_CPP)
#  undef signals
#  define signals protected
#endif


#include <IceUtil/UUID.h>
#include <QObject>
#include <QtCore>
#include <QTimer>
#include <qlog/qlog.h>

#include "handler.h"


class GazeboHandler : public Handler
{
Q_OBJECT
private:
#ifdef OLD_API
	gazebo::Client *client;
	gazebo::SimulationIface *simIface;
	gazebo::PositionIface *posIface;
#else
	libgazebo::Client *client;
	libgazebo::SimulationIface *simIface;
	libgazebo::PositionIface *posIface;
#endif
	
	RoboCompGenericBase::TBaseState bState;
	RoboCompDifferentialRobot::TMechParams mechParams;	

	float r00,r01,r10,r11,t0,t1,alpha;	//rotation and translation matrix
	
	float antX, antZ, antYaw;

	
public:
	GazeboHandler(RoboCompDifferentialRobot::TMechParams params);
	~GazeboHandler();

	void initMutex(QMutex *m);

	//Speed
	bool setSpeedBase(float adv,float rot);
	bool stopBase();
	//Params
	RoboCompDifferentialRobot::TMechParams getMechParams();
	RoboCompGenericBase::TBaseState getBaseState();
	//odometer
	bool resetOdometer();
	void correctOdometer(float x, float z, float alpha) { bState.correctedX+=x; bState.correctedZ+=z; bState.correctedAlpha+=alpha; }
	bool setOdometer(RoboCompGenericBase::TBaseState _bState);
	void compute();
private:
	void readMechParams();	

};

#endif

#endif

