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
#include "camerabusI.h"
/**
* \brief Default constructor
*/
CameraBusI::CameraBusI(Worker *_worker, QObject *parent) : QObject(parent)
{
	worker = _worker;
//	mu = worker->mutex;       // Shared worker mutex
	// Component initialization...
}

/**
* \brief Default destructor
*/
CameraBusI::~CameraBusI()
{
	// Free component resources here
}

// Component functions, implementation
/**
 * \brief Return array of CameraParams holding a struct for each camera in the bus
 * @return CameraParamList List of structs contain all params for each camera in the bus
 */
CameraParamsList CameraBusI::getAllCameraParams( const Ice::Current&) 
{ 
	return worker->cameraParamsList;
}
/**
 * \brief Return bus params
 * @return BusParams Struct contains bus configuration params
 */
BusParams CameraBusI::getBusParams( const Ice::Current&) 
{
	return worker->busparams;  
}
/**
 * \brief Return an image in the format specific in format parameter 
 * @param string Camera name in the bus
 * @param Format Image mode and dimensions
 * @return Image Return an image in the format requested
 */
void CameraBusI::getImage(const string& camera, const Format& format, Image& image, const Ice::Current&) 
{ 
	worker->getImage(camera,format,image);
}
/**
 * \brief Return an image in the format specific in format parameter, head and bState positions
 * @param string Camera name in the bus
 * @param Format Image mode and dimensions
 * @return Image Return an image in the format requested
 * @return THeadState Struct contains head position
 * @return TBaseState Struct contains base position compute by odometry
 */
void CameraBusI::getImageAndStates(const string &camera,const Format &format, Image& image, RoboCompJointMotor::MotorStateMap &hState, RoboCompDifferentialRobot::TBaseState &bState, const Ice::Current& )
{
	worker->getBaseState(bState);
	worker->getHeadState(hState);
	worker->getImage(camera,format,image);
}

/**
 * \brief Return  a vector of images in the format specific in format parameter 
 * @param string CameraList List contains cameras names in the bus
 * @param Format Image mode and dimensions
 * @param bool If the parameter all is true, then one frame from all cameras is returned without checking the list "cameraList"
 * @return Image Return  a vector of Images in the format requested
 */
void CameraBusI::getSyncImages(const CameraList& cameraList, const Format& format, bool all, ImageList& imagelist, const Ice::Current&) 
{ 
	worker->getSyncImages(cameraList,format,all,imagelist);
}

/**
 * \brief Return  a vector of images in the format specific in format parameter, head and bState positions
 * @param string CameraList List contains cameras names in the bus
 * @param Format Image mode and dimensions
 * @param bool If the parameter all is true, then one frame from all cameras is returned without checking the list "cameraList"
 * @return Image Return  a vector of Images in the format requested
 * @return THeadState Struct contains head position
 * @return TBaseState Struct contains base position compute by odometry
 */
void CameraBusI::getSyncImagesAndStates(const CameraList& cameraList, const Format& format, bool all, ImageList& imagelist, RoboCompJointMotor::MotorStateMap &hState,RoboCompDifferentialRobot::TBaseState &bState, const Ice::Current & )
{
	worker->getBaseState(bState);
	worker->getHeadState(hState);
	worker->getSyncImages(cameraList,format,all,imagelist);
}

