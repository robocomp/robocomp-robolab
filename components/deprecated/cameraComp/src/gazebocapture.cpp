/*
 *    Copyright (C) 2009-2010 by RoboLab - University of Extremadura
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
#ifdef COMPILE_GAZEBO

#include "gazebocapture.h"

GazeboCapture::GazeboCapture()
{
	client = new gazebo::Client();
	simIface = new gazebo::SimulationIface();
	camIface = new gazebo::CameraIface();
	_finished=false;
}


GazeboCapture::~GazeboCapture()
{
// 	simIface->Close(client);
// 	camIface->Close(client);
}



bool GazeboCapture::init(RoboCompCamera::TCamParams& params_, RoboCompJointMotor::JointMotorPrx head_, RoboCompDifferentialRobot::DifferentialRobotPrx base_)
{
	qDebug() << "Initiating GazeboCapture...";
	params = params_;
	head=head_;
	base=base_;
// 	jointmotor = jointmotor_;

	int serverId = 0;
	/// Check configuration file parameters
	if (params.device.c_str() == NULL ) qFatal ( "GazeboCapture::init() fatal error: device is NULL" );
	if (params.width == 0 || params.height == 0 )
	{
		qFatal ("GazeboCapture::init() fatal error: Frame size is 0");
		return -1;
	}

	QString deviceString = QString::fromStdString(params.device.c_str());
	QStringList deviceSides = deviceString.split("/", QString::SkipEmptyParts);
	if (deviceSides.size() != 2)
		qFatal("GazeboCapture::init() fatal error: device must be interface1,interface2,../simulation (e.g camera_iface_0:default or camera_iface_0,camera_iface_1/default)");
	QString clientSim = deviceSides[1];
	QStringList deviceList = deviceSides[0].split(",", QString::SkipEmptyParts);
	numCameras = deviceList.size();

	printf("simulation: %s\n", clientSim.toLocal8Bit().constData());
	for (int i=0; i< numCameras; i++)
	{
		printf("camera %d: %s\n", i, deviceList[i].toLocal8Bit().constData());
	}

	/// Alloc data types for every camera
	camIface = new gazebo::CameraIface[numCameras];
	imgVectors = new std::vector<uint8_t>[numCameras];
	sizes = new int[numCameras];

	/// Connect to the libgazebo server
	qDebug() << "Camera::Gazebo::initialize()";
	/// Connect to the libgazebo server
	try
	{
		client->ConnectWait(serverId, GZ_CLIENT_ID_USER_FIRST);
	}
	catch (std::string e)
	{
		qFatal("Gazebo error: Unable to connect\n");
	}
	/// Open the Simulation Interface
	try
	{
		simIface->Open(client, clientSim.toLocal8Bit().constData());
	}
	catch (std::string e)
	{
		qFatal("Gazebo error: Unable to connect to the sim interface\n");
	}

	for (int i=0; i< numCameras; i++)
	{
		printf("Opening camera #%d: %s\n", i, deviceList[i].toLocal8Bit().constData());
		/// Open the Camera interface
		try
		{
			qDebug() << "Connecting to camera " << deviceList[i].toLocal8Bit().constData();
			camIface[i].Open(client, deviceList[i].toLocal8Bit().constData());
		}
		catch (std::string e)
		{
			qFatal("Gazebo error: Unable to connect to the camera interface\n");
		}
		qDebug() << "Connected to camera " << deviceList[i].toLocal8Bit().constData();
	}

	// In gazebo, we should not write the values but READ them
	params.width = camIface[0].data->width;
	params.height = camIface[0].data->height;
	params.FPS = 15;
	params.mode = "MODE_320x240_RGB";
	params.leftCamera = 0;
	params.rightCamera = 1;
	params.bothCameras = 5;
	if (params.numCams != numCameras)
		qFatal("numCameras != number of cameras");


	// IPP related variables
	imageSize.width = params.width;
	imageSize.height = params.height;
	imageRoi.width = imageSize.width;
	imageRoi.height = imageSize.height;
	imageRoi.x = imageRoi.y = 0;

	printf("GazeboCapture()\n");
	return true;
}


void GazeboCapture::grab(  )
{
	if (params.talkToJointMotor==true)
	{
		try
		{
			RoboCompJointMotor::MotorStateMap map;
			head->getAllMotorState(map);
			hStateBefore.motorsState = map;
			hStateBefore.isMoving = map["neck"].isMoving or map["tilt"].isMoving or map["leftPan"].isMoving or map["rightPan"].isMoving;
		}
		catch (const Ice::Exception& ex)
		{
			std::cout << ex << "CameraComp - Error reading JointMotor state"<< std::endl;
		}
	}
	if ( params.talkToBase==true )
	{
		try { base->getBaseState (bStateBefore); }
		catch ( const Ice::Exception& ex ) { std::cout << ex << "BaseComp - Error reading Base state" << std::endl; }
	}

	mu->lock();
	for (int i=0; i<numCameras; ++i)
	{
		camIface[i].Lock(1);
		uint32_t size = camIface[i].data->width * camIface[i].data->height * 3;
// 		std::cout << "_" << size << "_" << std::endl;
		if (imgVectors[i].size() < size)
		{
			imgVectors[i].resize(size);
			sizes[i] = size;
		}
		uint8_t *src = camIface[i].data->image;
		uint8_t *dst = &(imgVectors[i])[0];
		for (uint ee=size/3; ee>0; ee--)
		{
			dst[0] = src[2];
			dst[1] = src[1];
			dst[2] = src[0];
			dst+=3;
			src+=3;
		}
		camIface[i].Unlock();




	}


	if (params.talkToJointMotor==true)
	{
		try
		{
			RoboCompJointMotor::MotorStateMap map;
			head->getAllMotorState(map);
			hStateAfter.motorsState = map;
			hStateAfter.isMoving = map["neck"].isMoving or map["tilt"].isMoving or map["leftPan"].isMoving or map["rightPan"].isMoving;
		}
		catch (const Ice::Exception& ex)
		{
			std::cout << ex << "CameraComp - Error reading JointMotor state"<< std::endl;
		}
		hState=hStateAfter;
		for(RoboCompCommonHead::dmotorsState::const_iterator it = hStateAfter.motorsState.begin(); it != hStateAfter.motorsState.end(); ++it)
		{
// 			printf("%s %f\n", it->first.c_str(), it->second.pos);
			if(it->second.pos != hStateBefore.motorsState[it->first].pos)
			{
				hState.motorsState[it->first].isMoving=true;
				hState.isMoving=true;
			}
		}
	}
	if ( params.talkToBase==true )
	{
		try { base->getBaseState (bStateAfter); }
		catch ( const Ice::Exception& ex ) { std::cout << ex << "BaseComp - Error reading Base state" << std::endl; }
		
		bState=bStateAfter;
	}


	mu->unlock();

}


//* Run thread method

void GazeboCapture::run()
{
	while (!_finished)
	{
/*		if (params.talkToJointMotor==true)
		{
			try { head->getAllMotorState(hState); }
			catch (const Ice::Exception& ex) { qDebug() << "CameraComp - Error reading JointMotor state"; }
		}
		if ( params.talkToBase==true )
		{
			try { base->getBaseState(bState); }
			catch ( const Ice::Exception& ex ) { qDebug() << "BaseComp - Error reading Base state";}
		}*/
		grab();
		usleep(15000);
	}
	if (_finished)
	{
		printf("Finished thread\n");
	}
}



///Accessors for public interface

void GazeboCapture::getYUVPtr(uchar *buf, uchar cam, RoboCompCommonHead::THeadState& hState_, RoboCompDifferentialRobot::TBaseState& bState_)
{
	qFatal("not yet");
}

void GazeboCapture::getYRGBPtr(uchar *dest, uchar cam, RoboCompCommonHead::THeadState&,RoboCompDifferentialRobot::TBaseState&)
{
	qFatal("not yet");
}

void GazeboCapture::getYPtr(uchar *dest, uchar cam, RoboCompCommonHead::THeadState &hState_, RoboCompDifferentialRobot::TBaseState &bState_)
{
	mu->lock();
	hState_ = hState;
	bState_ = bState;
	ippiRGBToGray_8u_C3C1R(&(imgVectors[cam])[0], params.width*3, dest, params.width, imageSize);
	mu->unlock();
}

void GazeboCapture::getYLogPolarPtr(uchar *dest, uchar cam, RoboCompCommonHead::THeadState& hState_, RoboCompDifferentialRobot::TBaseState& bState_)
{
	qFatal("not yet");
}

void GazeboCapture::getRGBPackedPtr( uchar *dest, uchar cam, RoboCompCommonHead::THeadState& hState_, RoboCompDifferentialRobot::TBaseState& bState_)
{
	mu->lock();
	hState_ = hState;
	bState_ = bState;
	memcpy(dest, &(imgVectors[cam])[0], sizes[cam]);
	mu->unlock();
}


#endif
