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
#if COMPILE_FLYCAPTURE==1

#ifndef FLYCAPTURE_H
#define FLYCAPTURE_H

#include <capturador.h>
#include <flycapture/FlyCapture2.h>

using namespace FlyCapture2;

class Flycapture : public Capturador
{
  public:
	Flycapture();
	~Flycapture();
	virtual void getYLogPolarPtr(uchar* , uchar , RoboCompCommonHead::THeadState& , RoboCompDifferentialRobot::TBaseState& );
	virtual void getRGBPackedPtr(uchar* , uchar , RoboCompCommonHead::THeadState& , RoboCompDifferentialRobot::TBaseState& );
	virtual void getYPtr(uchar* , uchar , RoboCompCommonHead::THeadState& , RoboCompDifferentialRobot::TBaseState& );
	virtual void getYRGBPtr(uchar* , uchar , RoboCompCommonHead::THeadState& , RoboCompDifferentialRobot::TBaseState& );
	virtual void getYUVPtr(uchar* , uchar , RoboCompCommonHead::THeadState& , RoboCompDifferentialRobot::TBaseState& );
	virtual void run();
	virtual bool init(RoboCompCamera::TCamParams & params , RoboCompJointMotor::JointMotorPrx head_, RoboCompDifferentialRobot::DifferentialRobotPrx base_);

  private:
	void PrintBuildInfo();
	void PrintCameraInfo( CameraInfo* pCamInfo );
	void PrintError( Error error );
	void PrintFormat7Capabilities( Format7Info fmt7Info );
	bool grab();
	void cleanup();

	Error error;
	BusManager busMgr;
	Camera** ppCameras;
	Mode fmt7Mode;
	PixelFormat fmt7PixFmt;
	Image image;
	RoboCompCamera::TCamParams params;
	RoboCompJointMotor::JointMotorPrx head;
	RoboCompDifferentialRobot::DifferentialRobotPrx base;

};

#endif // FLYCAPTURE_H
#endif