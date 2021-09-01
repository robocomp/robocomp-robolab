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

#include <DifferentialRobot.h>
#include <FullPoseEstimation.h>
#include <FullPoseEstimationPub.h>
#include <GenericBase.h>


#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100


using TuplePrx = std::tuple<RoboCompDifferentialRobot::DifferentialRobotPrxPtr,RoboCompGenericBase::GenericBasePrxPtr,RoboCompFullPoseEstimationPub::FullPoseEstimationPubPrxPtr>;


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


	RoboCompDifferentialRobot::DifferentialRobotPrxPtr differentialrobot_proxy;
	RoboCompGenericBase::GenericBasePrxPtr genericbase_proxy;
	RoboCompFullPoseEstimationPub::FullPoseEstimationPubPrxPtr fullposeestimationpub_pubproxy;

	virtual RoboCompFullPoseEstimation::FullPoseEuler FullPoseEstimation_getFullPoseEuler() = 0;
	virtual RoboCompFullPoseEstimation::FullPoseMatrix FullPoseEstimation_getFullPoseMatrix() = 0;
	virtual void FullPoseEstimation_setInitialPose(float x, float y, float z, float rx, float ry, float rz) = 0;

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
