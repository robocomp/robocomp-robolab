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
#include "fullposeestimationI.h"

FullPoseEstimationI::FullPoseEstimationI(GenericWorker *_worker)
{
	worker = _worker;
}


FullPoseEstimationI::~FullPoseEstimationI()
{
}


RoboCompFullPoseEstimation::FullPoseEuler FullPoseEstimationI::getFullPoseEuler(const Ice::Current&)
{
	return worker->FullPoseEstimation_getFullPoseEuler();
}

RoboCompFullPoseEstimation::FullPoseMatrix FullPoseEstimationI::getFullPoseMatrix(const Ice::Current&)
{
	return worker->FullPoseEstimation_getFullPoseMatrix();
}

void FullPoseEstimationI::setInitialPose(float x, float y, float z, float rx, float ry, float rz, const Ice::Current&)
{
	worker->FullPoseEstimation_setInitialPose(x, y, z, rx, ry, rz);
}

