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
#include "lidar3dI.h"

Lidar3DI::Lidar3DI(GenericWorker *_worker, const size_t id): worker(_worker), id(id)
{
	getLidarDataHandlers = {
		[this](auto a, auto b, auto c, auto d) { return worker->Lidar3D_getLidarData(a, b, c, d); }
	};

	getLidarDataArrayProyectedInImageHandlers = {
		[this](auto a) { return worker->Lidar3D_getLidarDataArrayProyectedInImage(a); }
	};

	getLidarDataByCategoryHandlers = {
		[this](auto a, auto b) { return worker->Lidar3D_getLidarDataByCategory(a, b); }
	};

	getLidarDataProyectedInImageHandlers = {
		[this](auto a) { return worker->Lidar3D_getLidarDataProyectedInImage(a); }
	};

	getLidarDataWithThreshold2dHandlers = {
		[this](auto a, auto b, auto c) { return worker->Lidar3D_getLidarDataWithThreshold2d(a, b, c); }
	};

}


Lidar3DI::~Lidar3DI()
{
}


RoboCompLidar3D::TData Lidar3DI::getLidarData(std::string name, float start, float len, int decimationDegreeFactor, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getLidarDataHandlers.size())
		return  getLidarDataHandlers[id](name, start, len, decimationDegreeFactor);
	else
		throw std::out_of_range("Invalid getLidarData id: " + std::to_string(id));

}

RoboCompLidar3D::TDataImage Lidar3DI::getLidarDataArrayProyectedInImage(std::string name, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getLidarDataArrayProyectedInImageHandlers.size())
		return  getLidarDataArrayProyectedInImageHandlers[id](name);
	else
		throw std::out_of_range("Invalid getLidarDataArrayProyectedInImage id: " + std::to_string(id));

}

RoboCompLidar3D::TDataCategory Lidar3DI::getLidarDataByCategory(RoboCompLidar3D::TCategories categories, Ice::Long timestamp, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getLidarDataByCategoryHandlers.size())
		return  getLidarDataByCategoryHandlers[id](categories, timestamp);
	else
		throw std::out_of_range("Invalid getLidarDataByCategory id: " + std::to_string(id));

}

RoboCompLidar3D::TData Lidar3DI::getLidarDataProyectedInImage(std::string name, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getLidarDataProyectedInImageHandlers.size())
		return  getLidarDataProyectedInImageHandlers[id](name);
	else
		throw std::out_of_range("Invalid getLidarDataProyectedInImage id: " + std::to_string(id));

}

RoboCompLidar3D::TData Lidar3DI::getLidarDataWithThreshold2d(std::string name, float distance, int decimationDegreeFactor, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getLidarDataWithThreshold2dHandlers.size())
		return  getLidarDataWithThreshold2dHandlers[id](name, distance, decimationDegreeFactor);
	else
		throw std::out_of_range("Invalid getLidarDataWithThreshold2d id: " + std::to_string(id));

}

