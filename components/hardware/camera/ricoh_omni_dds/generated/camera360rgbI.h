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
#ifndef CAMERA360RGB_H
#define CAMERA360RGB_H

// Ice includes
#include <Ice/Ice.h>
#include <Camera360RGB.h>
#include <QPointer>

#include "../src/specificworker.h"


class Camera360RGBI : public virtual RoboCompCamera360RGB::Camera360RGB
{
public:
	Camera360RGBI(GenericWorker *_worker, const size_t id);
	~Camera360RGBI();

	RoboCompCamera360RGB::TImage getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight, const Ice::Current&);

private:

	QPointer<GenericWorker> worker;
	size_t id;

	// Array handlers for each method
	std::array<std::function<RoboCompCamera360RGB::TImage(int&, int&, int&, int&, int&, int&)>, 1> getROIHandlers;

};

#endif
