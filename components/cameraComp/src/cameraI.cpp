/*
 *    Copyright (C) 2008-2010 by RoboLab - University of Extremadura
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
#include "cameraI.h"

CameraI::CameraI( RoboCompCamera::TCamParams _params, RoboCompJointMotor::JointMotorPrx head_, RoboCompDifferentialRobot::DifferentialRobotPrx base_, QObject *parent) : QObject(parent), params(_params)
{
	// Component initialization...
	qDebug() << "CameraI::CameraI() -> Entering CameraI, initializing component with driver " << QString::fromStdString(params.driver);

	driver = NULL;
	if (params.driver == "FIREWIRE")
		driver = new ieee1394capture();
	else if ( params.driver == "V4L2")
		driver = new V4l2capture();
	else if ( params.driver == "DUMMY")
		driver = new DummyCapture();
	else if ( params.driver == "PROSILICA")
#ifdef COMPILE_PROSILICA
	{
	  driver = new ProsilicaCapture();
	}
#else
	{
		qFatal("cameraComp fatal error: cameraComp was not compiled with Prosilica support. Exiting...");
	}
#endif
  else if ( params.driver == "FLYCAPTURE")
  #ifdef COMPILE_FLYCAPTURE
	  {
		driver = new Flycapture();
	  }
  #else
	  {
		  qFatal("camera1omp fatal error: cameraComp was not compiled with Flycapture support. Exiting...");
	  }
  #endif
	else if ( params.driver == "GAZEBO")
#ifdef COMPILE_GAZEBO
	{
		driver = new GazeboCapture();
	}
#else
	{
		qFatal("cameraComp fatal error: cameraComp was not compiled with Gazebo support. Exiting...");
	}
#endif
	else
		qFatal( "CameraI::CameraI() -> No driver %s available. Aborting ", params.driver.c_str() );

	tempImage.resize(params.width*params.height*6);
	tempImage2.resize(params.width*params.height*6);
	srcRectRoi.x = 0;
	srcRectRoi.y = 0;

	if (!driver->init(params, head_, base_))
		qFatal("CameraI::CameraI() -> Error initializing the driver!. Aborting..." );

	head=head_;
	base=base_;

	time.start();
	connect(&timer,SIGNAL(timeout()),this,SLOT(checkSleep()));
	driver->start();
	printf("CameraI::CameraI() -> Capturing...\n");
	timer.start(MAX_IDLE_TIME/2);
}

CameraI::~CameraI()
{


}

void CameraI::terminate()
{

	driver->_finished=true;
// 	driver->terminate();
	driver->wait();
	delete driver;
	qDebug()<<"stop";
}

void CameraI::checkSleep()
{
// 	if (time.elapsed() > MAX_IDLE_TIME && !driver->sleeped)
// 	{
// 		driver->sleeped=true;
// 		std::cout << "Camera sleeping...." << std::endl;
// 	}
}

void CameraI::throwHWException(QString msg)
{
	RoboCompCamera::HardwareFailedException ex;
	ex.what = msg.toStdString();
	throw ex;
}

/** ***********************************/
/** Interface methods implemetation ***/
/** ***********************************/

/**
 * Returns 2 Plane packed YUV422 Images as captured form the cameras
 * @param cam number of camera: 0->left : 1-> right : 5->both
 * @param roi RoboCompCamera::imgType uchar vector containing images
 * @param c Requesting Ice component identification
 */
void CameraI::getYUVImage( Ice::Int cam, imgType &roi, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState, const Ice::Current &)
{
	time.restart();
	if(driver->sleeped)
	{
		driver->sleeped=false;
// 		driver->start();
	}
	if (cam < params.numCams)
	{
		roi.resize( params.size * 2);
		driver->getYUVPtr(&roi[0], cam, hState, bState);
	}
	else if (cam==params.bothCameras && params.numCams==2) //two cameras
	{
		roi.resize(params.size * 4);
		driver->getYUVPtr(&roi[0], params.leftCamera, hState, bState);
		driver->getYUVPtr(&roi[0]+params.size*2, params.rightCamera, hState, bState);
	}
	else
	{
		printf("CameraComp - No Camera number %d.\n", cam);
		printf("cam:%d params.numCams:%d params.bothCameras:%d\n", cam, params.numCams, params.bothCameras);
		throwHWException(QString("CameraComp - No Camera number")+QString::number(cam)+".");
	}
}


/**
 * Returns Luminance image from cameras
 * @param cam number of camera: 0->left : 1-> right : 5->both
 * @param roi RoboCompCamera::imgType uchar vector containing images
 * @param c Requesting Ice component identification
 */
void CameraI::getYImage( Ice::Int cam, imgType &roi, RoboCompCommonHead::THeadState &hState, RoboCompDifferentialRobot::TBaseState &bState, const Ice::Current & )
{
	static IppiSize roiSize = {params.width, params.height};
	// If it was sleeping.
	time.restart();
	if(driver->sleeped)
	{
		driver->sleeped=false;
// 		driver->start();
	}
	if (cam < params.numCams)
	{
		roi.resize(params.size);
		driver->getYPtr(&roi[0], cam, hState, bState);
		if ((params.leftInverted==true and cam==params.leftCamera) or (params.rightInverted==true and cam==params.rightCamera))
			ippiMirror_8u_C1IR(&roi[0], params.width, roiSize, ippAxsBoth);
		if ((params.rotated and cam==params.leftCamera) or (params.rotated and cam==params.rightCamera))
		{
#ifdef USE_IPP
			ippiTranspose_8u_C1IR(&roi[0], params.width, roiSize);
#else
			qFatal("Operation not supported without IPP");
#endif
		}
	}
	else if (cam==params.bothCameras && params.numCams==2)
	{
		roi.resize( params.size * 2);

		driver->getYPtr(&roi[0], params.leftCamera, hState, bState);
		if (params.leftInverted == true)
			ippiMirror_8u_C1IR(&roi[0], params.width, roiSize, ippAxsBoth);
		if (params.rotated)
		{
#ifdef USE_IPP
			ippiTranspose_8u_C1IR(&roi[0], params.width, roiSize);
#else
			qFatal("Operation not supported without IPP");
#endif
		}
		driver->getYPtr(&roi[0]+params.size, params.rightCamera, hState, bState);
		if( params.rightInverted == true)
			ippiMirror_8u_C1IR(&roi[0]+params.size, params.width, roiSize, ippAxsBoth);
		if (params.rotated)
		{
#ifdef USE_IPP
			ippiTranspose_8u_C1IR(&roi[0]+params.size, params.width, roiSize);
#else
			qFatal("Operation not supported without IPP");
#endif
		}
	}
	else
	{
	  qDebug()<<"num cameras"<< params.numCams;
	  throwHWException(QString("CameraComp - No Camera number %l.").arg(cam));
	}
}

void CameraI::getYImageCR(Ice::Int cam, Ice::Int div, imgType &roi, RoboCompCommonHead::THeadState &hState, RoboCompDifferentialRobot::TBaseState &bState, const Ice::Current & ) {
	static IppiSize roiSize = {params.width, params.height};
	IppiSize destRoiSize;

	//If it was sleeping...
	time.restart();
	if(driver->sleeped)
	{
		driver->sleeped=false;
// 		driver->start();
	}

	roiSize.width=params.width;
	roiSize.height=params.height;

	if (cam < params.numCams )
	{
		driver->getYPtr(&tempImage[0], cam, hState, bState);

		if ((params.leftInverted==true and cam==params.leftCamera) or (params.rightInverted==true and cam==params.rightCamera))
			ippiMirror_8u_C1IR(&tempImage[0], params.width, roiSize, ippAxsBoth);
		if ((params.rotated and cam==params.leftCamera) or (params.rotated and cam==params.rightCamera))
		{
#ifdef USE_IPP
			ippiTranspose_8u_C1IR(&tempImage[0], params.width, roiSize);
#else
			qFatal("Operation not supported without IPP");
#endif
		}
	}
	else if (cam==params.bothCameras && params.numCams==2)
	{
		driver->getYPtr(&tempImage[0], 0, hState, bState);
		if (params.leftInverted)
			ippiMirror_8u_C1IR(&tempImage[0], params.width, roiSize, ippAxsBoth);
		if (params.rotated)
		{
#ifdef USE_IPP
			ippiTranspose_8u_C1IR(&tempImage[0], params.width, roiSize);
#else
			qFatal("Operation not supported without IPP");
#endif
		}
		driver->getYPtr(&tempImage[params.size], 1, hState, bState);
		if (params.rightInverted)
			ippiMirror_8u_C1IR(&tempImage[params.size], params.width, roiSize, ippAxsBoth);
		if (params.rotated)
		{
#ifdef USE_IPP
			ippiTranspose_8u_C1IR(&tempImage[params.size], params.width, roiSize);
#else
			qFatal("Operation not supported without IPP");
#endif
		}
		roiSize.height=2*params.height;
	}
	else
	{
		throwHWException(QString("CameraComp - No Camera number %l.").arg(cam));
		return;
	}

	destRoiSize.width = roiSize.width/div;
	destRoiSize.height = roiSize.height/div;
	srcRectRoi.width=roiSize.width;
	srcRectRoi.height=roiSize.height;
	ippiResize_8u_C1R(&tempImage[0], roiSize, roiSize.width, srcRectRoi, &tempImage2[0], destRoiSize.width, destRoiSize, (double)(1./((double)div)), (double)(1./((double)div)), IPPI_INTER_LINEAR);


	QByteArray array = qCompress((uint8_t *)&tempImage2[0], destRoiSize.width*destRoiSize.height, 9);

	if ((unsigned int)roi.size() != (unsigned int)array.size()) roi.resize(array.size());
	memcpy(&roi[0], array.data(), array.size());
}

/**
 * Returns Luminance Log-Polar transformed images from cameras.
 * @param cam number of camera: 0->left : 1-> right : 5->both
 * @param roi RoboCompCamera::imgType uchar vector containing images
 * @param c Requesting Ice component identification
 */
void CameraI::getYLogPolarImage(Ice::Int cam, imgType &roi, RoboCompCommonHead::THeadState &hState, RoboCompDifferentialRobot::TBaseState &bState, const Ice::Current &)
{
	RoboCompCamera::HardwareFailedException ex;

	time.restart();
	if(driver->sleeped)
	{
		driver->sleeped=false;
// 		driver->start();
	}

	if(  cam < params.numCams )
	{
		roi.resize( ecc * ang );
		driver->getYLogPolarPtr(&roi[0], cam, hState, bState);
	}
	else if(cam==params.bothCameras && params.numCams==2) //two cameras
	{
		roi.resize( ecc * ang * 2 );
		driver->getYLogPolarPtr(&roi[0], params.leftCamera, hState, bState);
		driver->getYLogPolarPtr(&roi[0], params.rightCamera, hState, bState);
	}
	else throwHWException(QString("CameraComp - No Camera number %l.").arg(cam));
}


/**
 * Return camera intrinsic parameters
 * @param
 * @return
 */
TCamParams CameraI::getCamParams(const Ice::Current &)
{
	TCamParams paramsMod = params;
	if (params.rotated)
	{
		paramsMod.width = params.height;
		paramsMod.height = params.width;
	}
	return paramsMod;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Returns Luminance, Red, Green and Blue Images in separated planes
 * @param cam number of camera: 0->left : 1-> right : 5->both
 * @param roi RoboCompCamera::imgType uchar vector containing images
 * @param c Requesting Ice component identification
 */
void CameraI::getYRGBImage( Ice::Int cam, imgType &roi, RoboCompCommonHead::THeadState &hState, RoboCompDifferentialRobot::TBaseState &bState, const Ice::Current & )
{
	//static IppiSize roiSize = {params.width, params.height};
	time.restart();
	if(driver->sleeped)
	{
		driver->sleeped=false;
// 		driver->start();
	}

	if(  cam < params.numCams )
	{
		roi.resize( params.size * 4 );
		driver->getYRGBPtr(&roi[0], cam, hState, bState);
	}
	else if(cam==params.bothCameras && params.numCams==2) //two cameras
	{
	/*	roi.resize( params.size * 8);
		driver->getYRGBPtr(&roi[0], params.leftCamera, hState, bState);
		if( params.leftInverted == true)
			ippiMirror_8u_C3IR(&roi[0], params.width*4, roiSize, ippAxsBoth);
		driver->getYRGBPtr(&roi[0]+params.size*4, params.rightCamera, hState, bState);
		if( params.rightInverted == true)
			ippiMirror_8u_C3IR(&roi[0]+params.size*4, params.width*4, roiSize, ippAxsBoth);*/
	}
	else throwHWException(QString("CameraComp - No Camera number %l.").arg(cam));
}

/**
 * Returns 2 Plane packed YUV422 Images as captured form the cameras
 * @param cam number of camera: 0->left : 1-> right : 5->both
 * @param roi RoboCompCamera::imgType uchar vector containing images
 * @param c Requesting Ice component identification
 */
void CameraI::getRGBPackedImage(Ice::Int cam, imgType &roi, RoboCompCommonHead::THeadState &hState, RoboCompDifferentialRobot::TBaseState &bState, const Ice::Current &)
{
	static IppiSize roiSize = {params.width, params.height};
	time.restart();
	if(driver->sleeped)
	{
		driver->sleeped=false;
// 		driver->start();
	}

	if (cam < params.numCams)
	{
		roi.resize( params.size * 3);

		if ((params.rotated and cam==params.leftCamera) or (params.rotated and cam==params.rightCamera))
		{
			driver->getRGBPackedPtr(&tempImage[0], cam, hState, bState);
			ippiMirror_8u_C3IR(&tempImage[0], params.width*3, roiSize, ippAxsHorizontal);
#ifdef USE_IPP
			ippiTranspose_8u_C3R(&tempImage[0], params.width*3, &roi[0], params.height*3, roiSize);
#else
			qFatal("Operation not supported without IPP");
#endif
		}
		else
			driver->getRGBPackedPtr(&roi[0], cam, hState, bState);

		if ((params.leftInverted==true and cam==params.leftCamera) or (params.rightInverted==true and cam==params.rightCamera))
			ippiMirror_8u_C3IR(&roi[0], params.width*3, roiSize, ippAxsBoth);

	}
	else if (cam==params.bothCameras && params.numCams==2) //two camaras
	{
		roi.resize( params.size * 6);

		if (params.rotated)
		{
			driver->getRGBPackedPtr(&tempImage[0], params.leftCamera, hState, bState);
			ippiMirror_8u_C3IR(&tempImage[0], params.width*3, roiSize, ippAxsHorizontal);
#ifdef USE_IPP
			ippiTranspose_8u_C3R(&tempImage[0], params.width*3, &roi[0], params.height*3, roiSize);
#else
			qFatal("Operation not supported without IPP");
#endif
		}
		else
			driver->getRGBPackedPtr(&roi[0], params.leftCamera, hState, bState);
		if (params.leftInverted == true)
			ippiMirror_8u_C3IR(&roi[0], params.width*3, roiSize, ippAxsBoth);


		if (params.rotated)
		{
			driver->getRGBPackedPtr(&tempImage[0], params.rightCamera, hState, bState);
			ippiMirror_8u_C3IR(&tempImage[0], params.width*3, roiSize, ippAxsHorizontal);
#ifdef USE_IPP
			ippiTranspose_8u_C3R(&tempImage[0], params.width*3, &roi[params.size*3], params.height*3, roiSize);
#else
			qFatal("Operation not supported without IPP");
#endif
		}
		else
			driver->getRGBPackedPtr(&roi[params.size*3], params.rightCamera, hState, bState);

		if( params.rightInverted == true)
			ippiMirror_8u_C3IR(&roi[params.size*3], params.width*3, roiSize, ippAxsBoth);
	}
	else	throwHWException(QString("CameraComp - No Camera number %l.").arg(cam));

}


void CameraI::setInnerImage(const imgType &roi, const Ice::Current &) {
	DummyCapture *dummy = dynamic_cast<DummyCapture*>(driver);
	if (dummy) {
		dummy->setImage((unsigned char *)&roi[0], roi.size());
	}
}
