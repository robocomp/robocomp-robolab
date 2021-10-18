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
#ifndef GENERICWORKER_H
#define GENERICWORKER_H

#include "config.h"
#include <stdint.h>
#include <qlog/qlog.h>
#include <CommonBehavior.h>

#include <BatteryStatus.h>
#include <DifferentialRobot.h>
#include <GenericBase.h>
#include <GiraffButton.h>
#include <JointMotorSimple.h>


#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100


using TuplePrx = std::tuple<>;


class GenericWorker : public QObject
{
Q_OBJECT
public:
	GenericWorker(TuplePrx tprx);
	virtual ~GenericWorker();
	virtual void killYourSelf();
	virtual void setPeriod(int p);

	virtual bool setParams(RoboCompCommonBehavior::ParameterList params) = 0;
	QMutex *mutex;



	virtual RoboCompBatteryStatus::TBattery BatteryStatus_getBatteryState() = 0;
	virtual void DifferentialRobot_correctOdometer(int x, int z, float alpha) = 0;
	virtual void DifferentialRobot_getBasePose(int &x, int &z, float &alpha) = 0;
	virtual void DifferentialRobot_getBaseState(RoboCompGenericBase::TBaseState &state) = 0;
	virtual void DifferentialRobot_resetOdometer() = 0;
	virtual void DifferentialRobot_setOdometer(RoboCompGenericBase::TBaseState state) = 0;
	virtual void DifferentialRobot_setOdometerPose(int x, int z, float alpha) = 0;
	virtual void DifferentialRobot_setSpeedBase(float adv, float rot) = 0;
	virtual void DifferentialRobot_stopBase() = 0;
	virtual RoboCompGiraffButton::Botones GiraffButton_getBotonesState() = 0;
	virtual RoboCompJointMotorSimple::MotorParams JointMotorSimple_getMotorParams(std::string motor) = 0;
	virtual RoboCompJointMotorSimple::MotorState JointMotorSimple_getMotorState(std::string motor) = 0;
	virtual void JointMotorSimple_setPosition(std::string name, RoboCompJointMotorSimple::MotorGoalPosition goal) = 0;
	virtual void JointMotorSimple_setVelocity(std::string name, RoboCompJointMotorSimple::MotorGoalVelocity goal) = 0;
	virtual void JointMotorSimple_setZeroPos(std::string name) = 0;

protected:

	QTimer timer;
	int Period;

private:


public slots:
	virtual void compute() = 0;
	virtual void initialize(int period) = 0;
	
signals:
	void kill();
};

#endif
