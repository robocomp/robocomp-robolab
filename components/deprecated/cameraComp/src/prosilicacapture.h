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
#if COMPILE_PROSILICA==1
#ifndef PROSILICACAPTURE_H
#define PROSILICACAPTURE_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <PvApi.h>
#include <QtCore>
#include "capturador.h"
#include <Camera.h>
#include <JointMotor.h>
#include <DifferentialRobot.h>

#ifdef USE_IPP
#include <ipp.h>
#else
#include <ippWrapper.h>
#endif

#define FRAMESCOUNT 15


class ProsilicaCapture :public Capturador
{
  public:
	ProsilicaCapture();
	~ProsilicaCapture();

	typedef struct
	{
	  unsigned long   UID;
	  tPvHandle       Handle;
	  tPvFrame		  Frame;
	  bool            Abort;
	} tCamera;

	// global camera data
	tCamera *GCamera;

	bool init(RoboCompCamera::TCamParams & params_, RoboCompJointMotor::JointMotorPrx head_ , RoboCompDifferentialRobot::DifferentialRobotPrx base_ );
	void cleanup();
	void run();
	void getYUVPtr(uchar*, uchar, RoboCompCommonHead::THeadState&,RoboCompDifferentialRobot::TBaseState &);
	void getYRGBPtr(uchar*, uchar, RoboCompCommonHead::THeadState&, RoboCompDifferentialRobot::TBaseState&);
	void getYPtr(uchar*, uchar, RoboCompCommonHead::THeadState&, RoboCompDifferentialRobot::TBaseState&);
	void getYLogPolarPtr(uchar*, uchar, RoboCompCommonHead::THeadState&,RoboCompDifferentialRobot::TBaseState&);
	void getRGBPackedPtr(uchar*, uchar, RoboCompCommonHead::THeadState&,RoboCompDifferentialRobot::TBaseState&);

  private:
	void Sleep(unsigned int time);
	// wait for a camera to be plugged
	void WaitForCamera();
	// wait forever (at least until there is no more camera
	void WaitForEver();
	// get the first camera found
	bool CameraGrab();
	// open the camera
	bool CameraSetup();
	// setup and start streaming
	bool CameraStart() ;
	// stop streaming
	void CameraStop();
	// unsetup the camera
	void CameraUnsetup() ;

	bool grab();

};

#endif // PROSILICACAPTURE_H
#endif