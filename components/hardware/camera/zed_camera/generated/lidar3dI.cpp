/*
 *    Copyright (C) 2026 by YOUR NAME HERE
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
	getColorCloudDataHandlers = {
		[this]() -> RoboCompLidar3D::TColorCloudData {if (worker != nullptr) return worker->Lidar3D_getColorCloudData(); else throw std::runtime_error("Worker is null");}
	};

	getLidarDataHandlers = {
		[this](auto &a, auto &b, auto &c, auto &d) -> RoboCompLidar3D::TData {if (worker != nullptr) return worker->Lidar3D_getLidarData(a, b, c, d); else throw std::runtime_error("Worker is null");}
	};

	getLidarDataArrayProyectedInImageHandlers = {
		[this](auto &a) -> RoboCompLidar3D::TDataImage {if (worker != nullptr) return worker->Lidar3D_getLidarDataArrayProyectedInImage(a); else throw std::runtime_error("Worker is null");}
	};

	getLidarDataByCategoryHandlers = {
		[this](auto &a, auto &b) -> RoboCompLidar3D::TDataCategory {if (worker != nullptr) return worker->Lidar3D_getLidarDataByCategory(a, b); else throw std::runtime_error("Worker is null");}
	};

	getLidarDataProyectedInImageHandlers = {
		[this](auto &a) -> RoboCompLidar3D::TData {if (worker != nullptr) return worker->Lidar3D_getLidarDataProyectedInImage(a); else throw std::runtime_error("Worker is null");}
	};

	getLidarDataWithThreshold2dHandlers = {
		[this](auto &a, auto &b, auto &c) -> RoboCompLidar3D::TData {if (worker != nullptr) return worker->Lidar3D_getLidarDataWithThreshold2d(a, b, c); else throw std::runtime_error("Worker is null");}
	};

}

Lidar3DI::~Lidar3DI()
{
}

RoboCompLidar3D::TColorCloudData Lidar3DI::getColorCloudData(const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return getColorCloudDataHandlers.at(id)();
}

RoboCompLidar3D::TData Lidar3DI::getLidarData(std::string name, float start, float len, int decimationDegreeFactor, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return getLidarDataHandlers.at(id)(name, start, len, decimationDegreeFactor);
}

RoboCompLidar3D::TDataImage Lidar3DI::getLidarDataArrayProyectedInImage(std::string name, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return getLidarDataArrayProyectedInImageHandlers.at(id)(name);
}

RoboCompLidar3D::TDataCategory Lidar3DI::getLidarDataByCategory(RoboCompLidar3D::TCategories categories, Ice::Long timestamp, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return getLidarDataByCategoryHandlers.at(id)(categories, timestamp);
}

RoboCompLidar3D::TData Lidar3DI::getLidarDataProyectedInImage(std::string name, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return getLidarDataProyectedInImageHandlers.at(id)(name);
}

RoboCompLidar3D::TData Lidar3DI::getLidarDataWithThreshold2d(std::string name, float distance, int decimationDegreeFactor, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return getLidarDataWithThreshold2dHandlers.at(id)(name, distance, decimationDegreeFactor);
}