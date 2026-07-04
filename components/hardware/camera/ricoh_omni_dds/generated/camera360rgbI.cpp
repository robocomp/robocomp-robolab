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
#include "camera360rgbI.h"

Camera360RGBI::Camera360RGBI(GenericWorker *_worker, const size_t id): worker(_worker), id(id)
{
	getROIHandlers = {
		[this](auto &a, auto &b, auto &c, auto &d, auto &e, auto &f) -> RoboCompCamera360RGB::TImage {if (worker != nullptr) return worker->Camera360RGB_getROI(a, b, c, d, e, f); else throw std::runtime_error("Worker is null");}
	};

}

Camera360RGBI::~Camera360RGBI()
{
}

RoboCompCamera360RGB::TImage Camera360RGBI::getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return getROIHandlers.at(id)(cx, cy, sx, sy, roiwidth, roiheight);
}