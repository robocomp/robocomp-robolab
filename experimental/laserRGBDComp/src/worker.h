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

#include <QtGui>
#include <stdint.h>
#include <qlog/qlog.h>
#include <CommonBehavior.h>

#include <vector>

#include <OmniRobot.h>
#include <Laser.h>
#include <JointMotor.h>
#include <RGBD.h>
#include <RGBDBus.h>
#include <innermodel/innermodel.h>
#include <innermodel/innermodelreader.h>

#include <extendedRangeSensor/extendedRangeSensor.h>

#include "config.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


#define BASIC_PERIOD 100

#define ERROR_C 20.
#define ERROR_L 4000.
#define LIMITE 440.

using namespace std;


struct LaserRGBDInfo
{
	string id;
	bool bus;
	RoboCompRGBD::RGBDPrx proxyRGBD;
	RoboCompRGBDBus::RGBDBusPrx proxyRGBDBus;
	RoboCompRGBDBus::CameraParamsMap cameras;
	RoboCompRGBDBus::PointCloudMap protoPointClouds;
};

/**
       \brief
       @author authorname
*/

struct WorkerConfig
{
	std::string xmlpath;
	std::string laserBaseID;
	std::vector<LaserRGBDInfo> rgbdsVec;
	float minHeight;
	float minHeightNeg;
	float maxHeight;
	int LASER_SIZE;
	float MIN_LENGTH;
	float maxLength;
	float FOV;
	int DECIMATION_LEVEL;
	bool updateJoint;

	// laser-related
	std::string actualLaserID;
	bool useLaser;
};

class Worker  : public QObject
{
Q_OBJECT
public:
	Worker(RoboCompOmniRobot::OmniRobotPrx omnirobotprx, RoboCompJointMotor::JointMotorPrx jointmotorprx, RoboCompLaser::LaserPrx laserprx, WorkerConfig &wconfig);
	~Worker();
	//CommonBehavior
	void killYourSelf();
	void setPeriod(int p);
	bool setParams(RoboCompCommonBehavior::ParameterList params_);
	
	QMutex *mutex;                //Shared mutex with servant


	RoboCompLaser::TLaserData getLaserData();
	RoboCompLaser::TLaserData getLaserAndBStateData(RoboCompDifferentialRobot::TBaseState &bState); 
	RoboCompLaser::LaserConfData getLaserConfData();

public slots:
	 void compute(); //main method

private:
	int Period;
	float minHeight;
	float minHeightNeg;
	float maxHeight;
	int LASER_SIZE;
	float MIN_LENGTH;
	float maxLength;
	float FOV;
	float localFOV;
	bool updateJoint;
	int DECIMATION_LEVEL;

	QTimer timer;
	QString base, actualLaserID;
	InnerModel *innerModel;
	RoboCompOmniRobot::OmniRobotPrx omnirobot;
	RoboCompJointMotor::JointMotorPrx jointmotor;
	RoboCompLaser::LaserPrx laser;
	std::vector<LaserRGBDInfo> rgbds;

	RoboCompJointMotor::MotorStateMap hState;
	RoboCompDifferentialRobot::TBaseState bState;
	RoboCompDifferentialRobot::TBaseState bStateOut;
	RoboCompLaser::TLaserData *laserDataW;
	RoboCompLaser::TLaserData *laserDataR;
	RoboCompLaser::LaserConfData confData;
	
	ExtendedRangeSensor *extended;

	void updateInnerModel();
	int32_t angle2bin(double ang);
	void medianFilter();

	void writePCD(std::string path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	void writePCD_Y0(std::string path);
signals:
	void kill();
};

#endif
