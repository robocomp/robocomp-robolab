/*
 *    Copyright (C) 2016 by YOUR NAME HERE
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

#include "delfos.h"

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	void correctOdometer(const int x, const int z, const float alpha);
	void getBasePose(int &x, int &z, float &alpha);
	void resetOdometer();
	void setOdometer(const TBaseState &state);
	void getBaseState(TBaseState &state);
	void setOdometerPose(const int x, const int z, const float alpha);
	void stopBase();
	void setSpeedBase(const float advx, const float advz, const float rot);



private:
	void setWheels(QVec wheelVels_);
	void computeOdometry(bool forced=false);
	float R, l1, l2;
	QMat M_wheels_2_vels;
	QMat M_vels_2_wheels;
	// Odometry control
	QVec wheelVels;
	float angle, x, z;
	float corrAngle, corrX, corrZ;
	InnerModel *innermodel;
	InnerModelTransform *backPose, *newPose;
	InnerModelTransform *corrBackPose, *corrNewPose;
public slots:
	void compute();


private:
	Delfos *delfos;
	double getElapsedSeconds(bool clear = false);
};

#endif



	
