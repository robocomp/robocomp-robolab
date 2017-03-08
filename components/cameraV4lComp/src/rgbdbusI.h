/*
 *    Copyright (C) 2017 by YOUR NAME HERE
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
#ifndef RGBDBUS_H
#define RGBDBUS_H

// Ice includes
#include <Ice/Ice.h>
#include <RGBDBus.h>

#include <config.h>
#include "genericworker.h"

using namespace RoboCompRGBDBus;

class RGBDBusI : public virtual RoboCompRGBDBus::RGBDBus
{
public:
	RGBDBusI(GenericWorker *_worker);
	~RGBDBusI();
	
	CameraParamsMap getAllCameraParams(const Ice::Current&);
	void getPointClouds(const CameraList  &cameras,  PointCloudMap  &clouds, const Ice::Current&);
	void getImages(const CameraList  &cameras,  ImageMap  &images, const Ice::Current&);
	void getProtoClouds(const CameraList  &cameras,  PointCloudMap  &protoClouds, const Ice::Current&);
	void getDecimatedImages(const CameraList  &cameras, const int  decimation,  ImageMap  &images, const Ice::Current&);

private:

	GenericWorker *worker;

};

#endif
