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
#ifndef CAMARAI_H
#define CAMARAI_H

#include <string>
#include <iostream>

#include <QtCore>

#ifdef USE_IPP
#include <ipp.h>
#else
#include <ippWrapper.h>
#endif

#include <Camera.h>
#include <JointMotor.h>
#include <DifferentialRobot.h>

#include "capturador.h"

#include "ieee1394capture.h"
#include "v4l2capture.h"
#include "gazebocapture.h"
#include "dummycapture.h"
#include "prosilicacapture.h"
#include "flycapture.h"

#include <config.h>

#define MAX_IDLE_TIME 3000

using namespace RoboCompCamera;

/**
	\class CameraI <p>Servant for CameraComp. This class implements the methods of the public interface of CameraComp. It derives from Camera that is the Ice proxy that will be include by other remote components using this</p>
*/

class CameraI : public QObject , public virtual RoboCompCamera::Camera
{
Q_OBJECT
public:
	/**
	 *    The constructor takes the params structure and two proxies two BaseComp and JointMotorComp so it can stamp each frame with the kinematic state of the base and the head of the robot. This functionality can be disabled using config bool parameters: talkToBase and talkToCamMotion
	 * @param _params structure that holds the parameters read from the config file
	 * @param head_  proxy to CamMotionComp
	 * @param base_  proxy to BaseComp
	 * @param parent parent object in Qt run time class hierarchy.
	 */
	CameraI( RoboCompCamera::TCamParams _params, RoboCompJointMotor::JointMotorPrx head_ , RoboCompDifferentialRobot::DifferentialRobotPrx base_ , QObject *parent = 0 );
	~CameraI();

	/**
	* Returns 2 Plane packed YUV422 Images as captured form the cameras
	* @param cam number of camera: 0->left : 1-> right : 5->both
	* @param roi RoboCompCamera::imgType uchar vector containing images
	* @param c Requesting Ice component identification
	*/
	virtual void getYUVImage( Ice::Int, imgType&, RoboCompCommonHead::THeadState&, RoboCompDifferentialRobot::TBaseState&,const Ice::Current & );
	/**
	* Returns CameraComp current parameters
	* @param
	* @return  Structure TCamParams defined in Camera.ice holding camera parameters
	*/
	virtual TCamParams getCamParams(const Ice::Current&);

	/**
	* Returns Luminance plane from camera
	* @param cam number of camera: 0->left : 1-> right : 5->both
	* @param img RoboCompCamera::imgType uchar vector containing images
	* @param head_
	* @param base_
	* @param ice Requesting Ice component identification
	*/
	virtual void getYImage( Ice::Int cam, imgType & img, RoboCompCommonHead::THeadState & head_, RoboCompDifferentialRobot::TBaseState & base_, const Ice::Current & ice);

	/**
	* Returns Luminance plane from camera
	* @param cam number of camera: 0->left : 1-> right : 5->both
	* @param img RoboCompCamera::imgType uchar vector containing images
	* @param div Resizing divisor
	* @param head_
	* @param base_
	* @param ice Requesting Ice component identification
	*/
	virtual void getYImageCR( Ice::Int cam, Ice::Int div, imgType & img, RoboCompCommonHead::THeadState & head_, RoboCompDifferentialRobot::TBaseState & base_, const Ice::Current & ice);

	/**
	* Returns Luminance Log-Polar transformed images from cameras.
	* @param cam number of camera: 0->left : 1-> right : 5->both
	* @param roi RoboCompCamera::imgType uchar vector containing images
	* @param c Requesting Ice component identification
	*/
	virtual void getYLogPolarImage(Ice::Int, imgType&, RoboCompCommonHead::THeadState & head_, RoboCompDifferentialRobot::TBaseState & base_, const Ice::Current &);

	/**
	* Returns Luminance, Red, Green and Blue Images in separated planes
	* @param cam number of camera: 0->left : 1-> right : 5->both
	* @param roi RoboCompCamera::imgType uchar vector containing images
	* @param c Requesting Ice component identification
	*/
	virtual void getYRGBImage( Ice::Int, imgType&, RoboCompCommonHead::THeadState & head_, RoboCompDifferentialRobot::TBaseState & base_, const Ice::Current & );

	/**
	* Returns 2 Plane packed YUV422 Images as captured form the cameras
	* @param cam number of camera: 0->left : 1-> right : 5->both
	* @param roi RoboCompCamera::imgType uchar vector containing images
	* @param c Requesting Ice component identification
	*/
	virtual void getRGBPackedImage( Ice::Int, imgType&, RoboCompCommonHead::THeadState & head_, RoboCompDifferentialRobot::TBaseState & base_, const Ice::Current &);

	/**
	*
	* @param roi RoboCompCamera::imgType uchar vector containing images
	* @param c Requesting Ice component identification
	*/
	virtual void setInnerImage(const imgType&, const Ice::Current &);

	void terminate();

private:
	int  ecc, ang;
	Capturador *driver;                       ///*<driver Handler implementation of Capturador.h
	QTime time;                               ///*<time QTime object used for checking if enough time has transcurred since last request
	QTimer timer;                             ///*<timer QTimer object for checking periodically if requests have arrived and, if not, sending the thread to sleep.
	RoboCompCamera::TCamParams params;
	RoboCompJointMotor::JointMotorPrx head;   ///*<head proxy to JointMotorComp
	RoboCompDifferentialRobot::DifferentialRobotPrx base;             ///*<base proxy to BaseComp

	imgType tempImage;
	imgType tempImage2;
	IppiRect srcRectRoi;
	RoboCompJointMotor::JointMotorPrx jointmotor;

	void throwHWException(QString msg);

protected slots:
	void checkSleep();
};

#endif
