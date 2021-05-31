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
#ifndef FULLPOSEESTIMATION_H
#define FULLPOSEESTIMATION_H

// Ice includes
#include <Ice/Ice.h>
#include <FullPoseEstimation.h>

#include <config.h>
#include "genericworker.h"


class FullPoseEstimationI : public virtual RoboCompFullPoseEstimation::FullPoseEstimation
{
public:
	FullPoseEstimationI(GenericWorker *_worker);
	~FullPoseEstimationI();

	RoboCompFullPoseEstimation::FullPoseEuler getFullPoseEuler(const Ice::Current&);
	RoboCompFullPoseEstimation::FullPoseMatrix getFullPoseMatrix(const Ice::Current&);
	void setInitialPose(float x, float y, float z, float rx, float ry, float rz, const Ice::Current&);

private:

	GenericWorker *worker;

};

#endif
