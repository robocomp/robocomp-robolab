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

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include "dynamixel_serial.h"
#include "dynamixel_sdk.h"
#include "handler.h"

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
    QMutex *w_mutex;

    RoboCompJointMotorSimple::MotorParams JointMotorSimple_getMotorParams(std::string motor);
	RoboCompJointMotorSimple::MotorState JointMotorSimple_getMotorState(std::string motor);
	void JointMotorSimple_setPosition(std::string name, RoboCompJointMotorSimple::MotorGoalPosition goal);
	void JointMotorSimple_setVelocity(std::string name, RoboCompJointMotorSimple::MotorGoalVelocity goal);
	void JointMotorSimple_setZeroPos(std::string name);


public slots:
	void compute();
	int startup_check();
	void initialize(int period);

private:
	bool startup_check_flag;
    RoboCompJointMotorSimple::MotorParams params;
    Dynamixel *handler;

};

#endif
