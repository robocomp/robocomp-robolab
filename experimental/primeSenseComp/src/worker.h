/*
 *    Copyright (C) 2006-2011 by RoboLab - University of Extremadura
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

#include <CommonBehavior.h>
#include <DifferentialRobot.h>
#include <JointMotor.h>
#include <RGBD.h>

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <QMutex>
#include <QTimer>

#include <ipp.h>

// using namespace RoboCompKinect;
// using namespace RoboCompRGBD;
using namespace RoboCompJointMotor;
using namespace RoboCompDifferentialRobot;


#define BASIC_PERIOD (1000/30)

using namespace std;



// struct WorkerConfig
// {
// 	bool depthOnly;
// };


/**
       \brief
       @author authorname
*/
class Worker  : public QObject
{
Q_OBJECT
public:
	Worker(int fps, DifferentialRobotPrx differential_, JointMotorPrx joint_);
	~Worker();

	//CommonBehavior
	void killYourSelf();
	void setPeriod(int p);
	bool setParams(RoboCompCommonBehavior::ParameterList params_);
	
	// RGBD interface
	RoboCompRGBD::TRGBDParams getRGBDParams();
	void setRegistration(RoboCompRGBD::Registration value);
	RoboCompRGBD::Registration getRegistration();
	void getImage( RoboCompRGBD::ColorSeq& color, RoboCompRGBD::DepthSeq& depth, RoboCompRGBD::PointSeq& points, RoboCompJointMotor::MotorStateMap &head, RoboCompDifferentialRobot::TBaseState& base );
	void getDepth(RoboCompRGBD::DepthSeq& depth, RoboCompJointMotor::MotorStateMap &head, RoboCompDifferentialRobot::TBaseState& base );
	void getRGB(RoboCompRGBD::ColorSeq& color, RoboCompJointMotor::MotorStateMap &head, RoboCompDifferentialRobot::TBaseState& base );
	void getXYZ(RoboCompRGBD::PointSeq& points, RoboCompJointMotor::MotorStateMap &head, RoboCompDifferentialRobot::TBaseState& base );
	void getData(RoboCompRGBD::imgType& rgbMatrix, RoboCompRGBD::depthType& distanceMatrix, RoboCompJointMotor::MotorStateMap &hState, RoboCompDifferentialRobot::TBaseState& bState);
	
	// Attributes
	QMutex *mutex;                //Shared mutex with servant
// 	WorkerConfig config;

	DifferentialRobotPrx differential;
	JointMotorPrx joint;
private:
	struct Data;
	Data* d;

	bool puntos;
	//luiky
	Ipp32f *depthLuiky;
	int step_depth;

public slots:
	void compute();
	void printFPS();

signals:
	void kill();
};

#endif

