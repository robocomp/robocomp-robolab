/*
 *    Copyright (C) 2023 by YOUR NAME HERE
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
#include "lidar3dI.h"

Lidar3DI::Lidar3DI(GenericWorker *_worker)
{
	worker = _worker;
}


Lidar3DI::~Lidar3DI()
{
}


RoboCompLidar3D::TData Lidar3DI::getLidarData(std::string name, float start, float len, int decimationDegreeFactor, const Ice::Current&)
{
	return worker->Lidar3D_getLidarData(name, start, len, decimationDegreeFactor);
}

RoboCompLidar3D::TDataImage Lidar3DI::getLidarDataArrayProyectedInImage(std::string name, const Ice::Current&)
{
	return worker->Lidar3D_getLidarDataArrayProyectedInImage(name);
}

RoboCompLidar3D::TData Lidar3DI::getLidarDataProyectedInImage(std::string name, const Ice::Current&)
{
	return worker->Lidar3D_getLidarDataProyectedInImage(name);
}

RoboCompLidar3D::TData Lidar3DI::getLidarDataWithThreshold2d(std::string name, float distance, int decimationDegreeFactor, const Ice::Current&)
{
	return worker->Lidar3D_getLidarDataWithThreshold2d(name, distance, decimationDegreeFactor);
}

