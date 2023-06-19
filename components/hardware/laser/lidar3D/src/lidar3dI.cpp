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


RoboCompLidar3D::TLidarData Lidar3DI::getLidarData(std::string name, int start, int len, int decimationfactor, const Ice::Current&)
{
	return worker->Lidar3D_getLidarData(name, start, len, decimationfactor);
}

