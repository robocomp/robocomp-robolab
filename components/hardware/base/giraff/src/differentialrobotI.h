/*
 *    Copyright (C) 2021 by YOUR NAME HERE
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
#ifndef DIFFERENTIALROBOT_H
#define DIFFERENTIALROBOT_H

// Ice includes
#include <Ice/Ice.h>
#include <DifferentialRobot.h>

#include <config.h>
#include "genericworker.h"


class DifferentialRobotI : public virtual RoboCompDifferentialRobot::DifferentialRobot
{
public:
	DifferentialRobotI(GenericWorker *_worker);
	~DifferentialRobotI();

	void correctOdometer(int x, int z, float alpha, const Ice::Current&);
	void getBasePose(int &x, int &z, float &alpha, const Ice::Current&);
	void getBaseState(RoboCompGenericBase::TBaseState &state, const Ice::Current&);
	void resetOdometer(const Ice::Current&);
	void setOdometer(RoboCompGenericBase::TBaseState state, const Ice::Current&);
	void setOdometerPose(int x, int z, float alpha, const Ice::Current&);
	void setSpeedBase(float adv, float rot, const Ice::Current&);
	void stopBase(const Ice::Current&);

private:

	GenericWorker *worker;

};

#endif
