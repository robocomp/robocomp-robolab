/*
 *    Copyright (C) 2025 by YOUR NAME HERE
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
#ifndef LIDAR3D_H
#define LIDAR3D_H

// Ice includes
#include <Ice/Ice.h>
#include <Lidar3D.h>

#include "../src/specificworker.h"


class Lidar3DI : public virtual RoboCompLidar3D::Lidar3D
{
public:
	Lidar3DI(GenericWorker *_worker, const size_t id);
	~Lidar3DI();

	RoboCompLidar3D::TData getLidarData(std::string name, float start, float len, int decimationDegreeFactor, const Ice::Current&);
	RoboCompLidar3D::TDataImage getLidarDataArrayProyectedInImage(std::string name, const Ice::Current&);
	RoboCompLidar3D::TDataCategory getLidarDataByCategory(RoboCompLidar3D::TCategories categories, Ice::Long timestamp, const Ice::Current&);
	RoboCompLidar3D::TData getLidarDataProyectedInImage(std::string name, const Ice::Current&);
	RoboCompLidar3D::TData getLidarDataWithThreshold2d(std::string name, float distance, int decimationDegreeFactor, const Ice::Current&);

private:

	GenericWorker *worker;
	size_t id;

	// Array handlers for each method
	std::array<std::function<RoboCompLidar3D::TData(std::string, float, float, int)>, 1> getLidarDataHandlers;
	std::array<std::function<RoboCompLidar3D::TDataImage(std::string)>, 1> getLidarDataArrayProyectedInImageHandlers;
	std::array<std::function<RoboCompLidar3D::TDataCategory(RoboCompLidar3D::TCategories, Ice::Long)>, 1> getLidarDataByCategoryHandlers;
	std::array<std::function<RoboCompLidar3D::TData(std::string)>, 1> getLidarDataProyectedInImageHandlers;
	std::array<std::function<RoboCompLidar3D::TData(std::string, float, int)>, 1> getLidarDataWithThreshold2dHandlers;

};

#endif
