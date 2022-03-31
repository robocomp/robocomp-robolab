/*
 *    Copyright (C) 2022 by YOUR NAME HERE
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

#if Qt5_FOUND
	#include <QtWidgets>
#else
	#include <QtGui>
#endif
#include <ui_mainUI.h>
#include <CommonBehavior.h>

#include <CameraRGBDSimple.h>
#include <CameraSimple.h>
#include <DifferentialRobot.h>
#include <FullPoseEstimation.h>
#include <GenericBase.h>
#include <JointMotorSimple.h>
#include <Laser.h>


#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100


using TuplePrx = std::tuple<RoboCompCameraRGBDSimple::CameraRGBDSimplePrxPtr,RoboCompCameraSimple::CameraSimplePrxPtr,RoboCompDifferentialRobot::DifferentialRobotPrxPtr,RoboCompFullPoseEstimation::FullPoseEstimationPrxPtr,RoboCompJointMotorSimple::JointMotorSimplePrxPtr,RoboCompLaser::LaserPrxPtr>;


class GenericWorker : public QWidget, public Ui_guiDlg
{
Q_OBJECT
public:
	GenericWorker(TuplePrx tprx);
	virtual ~GenericWorker();
	virtual void killYourSelf();
	virtual void setPeriod(int p);

	virtual bool setParams(RoboCompCommonBehavior::ParameterList params) = 0;
	QMutex *mutex;


	RoboCompCameraRGBDSimple::CameraRGBDSimplePrxPtr camerargbdsimple_proxy;
	RoboCompCameraSimple::CameraSimplePrxPtr camerasimple_proxy;
	RoboCompDifferentialRobot::DifferentialRobotPrxPtr differentialrobot_proxy;
	RoboCompFullPoseEstimation::FullPoseEstimationPrxPtr fullposeestimation_proxy;
	RoboCompJointMotorSimple::JointMotorSimplePrxPtr jointmotorsimple_proxy;
	RoboCompLaser::LaserPrxPtr laser_proxy;


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
