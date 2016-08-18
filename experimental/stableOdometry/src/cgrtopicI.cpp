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
#include "cgrtopicI.h"

CGRTopicI::CGRTopicI(GenericWorker *_worker, QObject *parent) : QObject(parent)
{
	worker = _worker;
	mutex = worker->mutex;       // Shared worker mutex
}


CGRTopicI::~CGRTopicI()
{
}

void CGRTopicI::newCGRPose(const float  poseUncertainty, const float  x, const float  z, const float  alpha, const Ice::Current&)
{
	worker->newCGRPose(poseUncertainty, x, z, alpha);
}

void CGRTopicI::newCGRCorrection(const float  poseUncertainty, const float  x1, const float  z1, const float  alpha1, const float  x2, const float  z2, const float  alpha2, const Ice::Current&)
{
	worker->newCGRCorrection(poseUncertainty, x1, z1, alpha1, x2, z2, alpha2);
}






