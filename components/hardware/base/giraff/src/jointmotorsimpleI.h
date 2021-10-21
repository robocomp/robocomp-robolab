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
#ifndef JOINTMOTORSIMPLE_H
#define JOINTMOTORSIMPLE_H

// Ice includes
#include <Ice/Ice.h>
#include <JointMotorSimple.h>

#include <config.h>
#include "genericworker.h"


class JointMotorSimpleI : public virtual RoboCompJointMotorSimple::JointMotorSimple
{
public:
	JointMotorSimpleI(GenericWorker *_worker);
	~JointMotorSimpleI();

	RoboCompJointMotorSimple::MotorParams getMotorParams(std::string motor, const Ice::Current&);
	RoboCompJointMotorSimple::MotorState getMotorState(std::string motor, const Ice::Current&);
	void setPosition(std::string name, RoboCompJointMotorSimple::MotorGoalPosition goal, const Ice::Current&);
	void setVelocity(std::string name, RoboCompJointMotorSimple::MotorGoalVelocity goal, const Ice::Current&);
	void setZeroPos(std::string name, const Ice::Current&);

private:

	GenericWorker *worker;

};

#endif
