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
#ifndef DIFFERENTIALROBOTI_H
#define DIFFERENTIALROBOTI_H

// QT includes
#include <QtCore/QObject>

// Ice includes
#include <Ice/Ice.h>
#include <DifferentialRobot.h>
#include <GenericBase.h>

#include <config.h>
#include "worker.h"

using namespace RoboCompDifferentialRobot;
using namespace RoboCompGenericBase;
/**
	\class DifferentialRobotI <p>Servant for DifferentialRobotComp. This class implements the methods of the public interface of DifferentialRobotComp.
	It derives from DifferentialRobot that is the Ice proxy that will be include by other remote components using this</p>
*/
class DifferentialRobotI : public QObject , public virtual RoboCompDifferentialRobot::DifferentialRobot
{
Q_OBJECT
public:
	DifferentialRobotI( Worker *_worker, QObject *parent = 0 );
	~DifferentialRobotI();
	
	void getBaseState(TBaseState &state, const Ice::Current & = Ice::Current());
	void getBasePose(int &x, int &z, float &alpha, const Ice::Current & = Ice::Current());
	void setSpeedBase(float adv, float rot, const Ice::Current & = Ice::Current());
	void stopBase( const Ice::Current & = Ice::Current());
	void resetOdometer( const Ice::Current & = Ice::Current());
	void setOdometer(const RoboCompGenericBase::TBaseState &state, const Ice::Current & = Ice::Current());
	void setOdometerPose(int x, int z, float alpha, const Ice::Current & = Ice::Current());
	void correctOdometer(int x, int z, float alpha, const Ice::Current & = Ice::Current());
	void getMechParams(RoboCompDifferentialRobot::TMechParams &mechParams, const Ice::Current & = Ice::Current());

private:
	void throwException(std::string msg);
	Worker *worker;	///*<worker Pointer to access worker methods. 
	QMutex *mutex;  ///*<mutex Object ensures mutual exclusion on access to variables
public slots:


};

#endif
