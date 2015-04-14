/*
 *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
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
#include "rgbdbusI.h"

RGBDBusI::RGBDBusI(GenericWorker *_worker, QObject *parent) : QObject(parent)
{
	worker = _worker;
	mutex = worker->mutex;       // Shared worker mutex
	// Component initialization...
}


RGBDBusI::~RGBDBusI()
{
	// Free component resources here
}

// Component functions, implementation
CameraParamsMap RGBDBusI::getAllCameraParams(const Ice::Current&){
	return worker->getAllCameraParams();
}

void RGBDBusI::getImages(const CameraList& cameras, ImageMap& images, const Ice::Current&){
	worker->getImages(cameras,images);
}

void RGBDBusI::getPointClouds(const CameraList& cameras, PointCloudMap& clouds, const Ice::Current&){
	worker->getPointClouds(cameras,clouds);
}

void RGBDBusI::getProtoClouds(const CameraList& cameras, PointCloudMap& protoClouds, const Ice::Current&){
	worker->getProtoClouds(cameras,protoClouds);
}

void RGBDBusI::getDecimatedImages(const CameraList& cameras, Ice::Int decimation, ImageMap& images, const Ice::Current&){
	worker->getDecimatedImages(cameras,decimation,images);
}


