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
#include "camera360rgbI.h"

Camera360RGBI::Camera360RGBI(GenericWorker *_worker)
{
	worker = _worker;
}


Camera360RGBI::~Camera360RGBI()
{
}


RoboCompCameraRGBDSimple::TImage Camera360RGBI::getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight, const Ice::Current&)
{
	return worker->Camera360RGB_getROI(cx, cy, sx, sy, roiwidth, roiheight);
}

