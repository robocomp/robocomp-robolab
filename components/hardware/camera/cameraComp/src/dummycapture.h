/*
 *    Copyright (C) 2010 by RoboLab - University of Extremadura
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
#ifndef DUMMYCAPTURE_H
#define DUMMYCAPTURE_H

#include <iostream>

#include <stdio.h>
#include <stdlib.h>

#include <QtCore>
#include <QImage>

#include <Camera.h>
#include <JointMotor.h>
#include <DifferentialRobot.h>

#include "capturador.h"

#ifdef USE_IPP
#include <ipp.h>
#else
#include <ippWrapper.h>
#endif

using namespace std;

class DummyCapture : public Capturador
{
Q_OBJECT
public:
	DummyCapture();
	~DummyCapture() {}
	bool init(RoboCompCamera::TCamParams &params_, RoboCompJointMotor::JointMotorPrx head_ , RoboCompDifferentialRobot::DifferentialRobotPrx base_);
	void run();
	bool grab() { return true; }
	void getYUVPtr(uchar*, uchar, RoboCompCommonHead::THeadState &,RoboCompDifferentialRobot::TBaseState &);
	void getYRGBPtr(uchar* , uchar, RoboCompCommonHead::THeadState &,RoboCompDifferentialRobot::TBaseState &);
	void getYPtr(uchar* , uchar, RoboCompCommonHead::THeadState & , RoboCompDifferentialRobot::TBaseState &);
	void getYLogPolarPtr(uchar* , uchar, RoboCompCommonHead::THeadState &,RoboCompDifferentialRobot::TBaseState &);
	void getRGBPackedPtr(uchar* , uchar, RoboCompCommonHead::THeadState &,RoboCompDifferentialRobot::TBaseState &);

	void setImage(unsigned char *image, int _size);

private:
	uchar *cap_buffer1;
	int size;

};

#endif
