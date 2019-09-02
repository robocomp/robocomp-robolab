/*
 *    Copyright (C)2019 by YOUR NAME HERE
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
#include "handler.h"
#include "robexhandler.h"
#include "gazebohandler.h"


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	void DifferentialRobot_correctOdometer(const int x, const int z, const float alpha);
	void DifferentialRobot_getBasePose(int &x, int &z, float &alpha);
	void DifferentialRobot_resetOdometer();
	void DifferentialRobot_setOdometer(const RoboCompGenericBase::TBaseState &state);
	void DifferentialRobot_getBaseState(RoboCompGenericBase::TBaseState &state);
	void DifferentialRobot_setOdometerPose(const int x, const int z, const float alpha);
	void DifferentialRobot_stopBase();
	void DifferentialRobot_setSpeedBase(const float adv, const float rot);
	void GenericBase_getBaseState(RoboCompGenericBase::TBaseState &state);
	void GenericBase_getBasePose(int &x, int &z, float &alpha);
	void JoystickAdapter_sendData(const TData &data);

	//utils
	float normalize(float X, float A, float B, float C, float D);
	
public slots:
	void compute();
	void initialize(int period);

private:
	Handler *handler;	
	InnerModel *innerModel;
	RoboCompDifferentialRobot::TMechParams params;
	RoboCompGenericBase::TBaseState bState;
};

#endif
