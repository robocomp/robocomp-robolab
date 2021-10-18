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
#include <innermodel/innermodel.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	RoboCompBatteryStatus::TBattery BatteryStatus_getBatteryState();
	void DifferentialRobot_correctOdometer(int x, int z, float alpha);
	void DifferentialRobot_getBasePose(int &x, int &z, float &alpha);
	void DifferentialRobot_getBaseState(RoboCompGenericBase::TBaseState &state);
	void DifferentialRobot_resetOdometer();
	void DifferentialRobot_setOdometer(RoboCompGenericBase::TBaseState state);
	void DifferentialRobot_setOdometerPose(int x, int z, float alpha);
	void DifferentialRobot_setSpeedBase(float adv, float rot);
	void DifferentialRobot_stopBase();
	RoboCompGiraffButton::Botones GiraffButton_getBotonesState();
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
	std::shared_ptr < InnerModel > innerModel;
	bool startup_check_flag;

};

#endif
