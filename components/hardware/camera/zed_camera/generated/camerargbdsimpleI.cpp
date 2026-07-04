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
#include "camerargbdsimpleI.h"

CameraRGBDSimpleI::CameraRGBDSimpleI(GenericWorker *_worker, const size_t id): worker(_worker), id(id)
{
	getAllHandlers = {
		[this](auto &a) -> RoboCompCameraRGBDSimple::TRGBD {if (worker != nullptr) return worker->CameraRGBDSimple_getAll(a); else throw std::runtime_error("Worker is null");}
	};

	getDepthHandlers = {
		[this](auto &a) -> RoboCompCameraRGBDSimple::TDepth {if (worker != nullptr) return worker->CameraRGBDSimple_getDepth(a); else throw std::runtime_error("Worker is null");}
	};

	getImageHandlers = {
		[this](auto &a) -> RoboCompCameraRGBDSimple::TImage {if (worker != nullptr) return worker->CameraRGBDSimple_getImage(a); else throw std::runtime_error("Worker is null");}
	};

	getPointsHandlers = {
		[this](auto &a) -> RoboCompCameraRGBDSimple::TPoints {if (worker != nullptr) return worker->CameraRGBDSimple_getPoints(a); else throw std::runtime_error("Worker is null");}
	};

}

CameraRGBDSimpleI::~CameraRGBDSimpleI()
{
}

RoboCompCameraRGBDSimple::TRGBD CameraRGBDSimpleI::getAll(std::string camera, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return getAllHandlers.at(id)(camera);
}

RoboCompCameraRGBDSimple::TDepth CameraRGBDSimpleI::getDepth(std::string camera, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return getDepthHandlers.at(id)(camera);
}

RoboCompCameraRGBDSimple::TImage CameraRGBDSimpleI::getImage(std::string camera, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return getImageHandlers.at(id)(camera);
}

RoboCompCameraRGBDSimple::TPoints CameraRGBDSimpleI::getPoints(std::string camera, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return getPointsHandlers.at(id)(camera);
}