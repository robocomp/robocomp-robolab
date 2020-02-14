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
#include "rgbdbusI.h"

RGBDBusI::RGBDBusI(GenericWorker *_worker)
{
	worker = _worker;
}


RGBDBusI::~RGBDBusI()
{
}

CameraParamsMap RGBDBusI::getAllCameraParams(const Ice::Current&)
{
	return worker->getAllCameraParams();
}

void RGBDBusI::getPointClouds(const CameraList  &cameras,  PointCloudMap  &clouds, const Ice::Current&)
{
	worker->getPointClouds(cameras, clouds);
}

void RGBDBusI::getImages(const CameraList  &cameras,  ImageMap  &images, const Ice::Current&)
{
	worker->getImages(cameras, images);
}

void RGBDBusI::getProtoClouds(const CameraList  &cameras,  PointCloudMap  &protoClouds, const Ice::Current&)
{
	worker->getProtoClouds(cameras, protoClouds);
}

void RGBDBusI::getDecimatedImages(const CameraList  &cameras, const int  decimation,  ImageMap  &images, const Ice::Current&)
{
	worker->getDecimatedImages(cameras, decimation, images);
}






