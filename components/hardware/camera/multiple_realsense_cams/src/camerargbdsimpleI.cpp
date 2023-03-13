/*
 *    Copyright (C) 2022 by YOUR NAME HERE
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

CameraRGBDSimpleI::CameraRGBDSimpleI(GenericWorker *_worker)
{
	worker = _worker;
}


CameraRGBDSimpleI::~CameraRGBDSimpleI()
{
}


RoboCompCameraRGBDSimple::TRGBD CameraRGBDSimpleI::getAll(std::string camera, const Ice::Current&)
{
	return worker->CameraRGBDSimple_getAll(camera);
}

RoboCompCameraRGBDSimple::TDepth CameraRGBDSimpleI::getDepth(std::string camera, const Ice::Current&)
{
	return worker->CameraRGBDSimple_getDepth(camera);
}

RoboCompCameraRGBDSimple::TImage CameraRGBDSimpleI::getImage(std::string camera, const Ice::Current&)
{
	return worker->CameraRGBDSimple_getImage(camera);
}

