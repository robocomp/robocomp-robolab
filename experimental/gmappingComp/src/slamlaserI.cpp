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
#include "slamlaserI.h"

SlamLaserI::SlamLaserI(GenericWorker *_worker)
{
	worker = _worker;
}


SlamLaserI::~SlamLaserI()
{
}

bool SlamLaserI::saveMap(const string  &path, const Ice::Current&)
{
	return worker->saveMap(path);
}

void SlamLaserI::getWholeGrid( GridMap  &map,  Pose2D  &pose, const Ice::Current&)
{
	worker->getWholeGrid(map, pose);
}

void SlamLaserI::initializeRobotPose(const Pose2D  &pose, const Ice::Current&)
{
	worker->initializeRobotPose(pose);
}

void SlamLaserI::getPartialGrid(const MapRect  &rect,  GridMap  &map,  Pose2D  &pose, const Ice::Current&)
{
	worker->getPartialGrid(rect, map, pose);
}






