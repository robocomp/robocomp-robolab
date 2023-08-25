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
#include "gpsubloxI.h"

GpsUbloxI::GpsUbloxI(GenericWorker *_worker)
{
	worker = _worker;
}


GpsUbloxI::~GpsUbloxI()
{
}


RoboCompGpsUblox::DatosGPS GpsUbloxI::getData(const Ice::Current&)
{
	return worker->GpsUblox_getData();
}

void GpsUbloxI::setInitialPose(float x, float y, const Ice::Current&)
{
	worker->GpsUblox_setInitialPose(x, y);
}

