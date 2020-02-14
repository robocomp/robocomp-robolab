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

#ifndef GAZEBOCAPTURE_H
#define GAZEBOCAPTURE_H

#include <vector>

#if defined(signals) && defined(QOBJECTDEFS_H) && !defined(QT_MOC_CPP)
#  undef signals
#  define signals signals
#endif
#undef UNPAUSE
#undef RESET
#undef STEP
#include <gazebo/gazebo.h>
// #include <gazebo/GazeboError.hh>
namespace boost
{
  namespace signalslib = signals;
}
#if defined(signals) && defined(QOBJECTDEFS_H) && !defined(QT_MOC_CPP)
#  undef signals
#  define signals protected
#endif


class Client;

#include "capturador.h"

class GazeboCapture : public Capturador
{
public:
	GazeboCapture();
	~GazeboCapture();
	bool init(RoboCompCamera::TCamParams& params_, RoboCompJointMotor::JointMotorPrx, RoboCompDifferentialRobot::DifferentialRobotPrx);
	void run();
	void getYUVPtr(uchar*, uchar cam, RoboCompCommonHead::THeadState&,RoboCompDifferentialRobot::TBaseState&);
	void getYRGBPtr(uchar* , uchar cam, RoboCompCommonHead::THeadState&,RoboCompDifferentialRobot::TBaseState&);
	void getYPtr(uchar* , uchar cam, RoboCompCommonHead::THeadState&, RoboCompDifferentialRobot::TBaseState &);
	void getYLogPolarPtr(uchar* , uchar cam, RoboCompCommonHead::THeadState&,RoboCompDifferentialRobot::TBaseState&);
	void getRGBPackedPtr(uchar* , uchar cam, RoboCompCommonHead::THeadState&,RoboCompDifferentialRobot::TBaseState&);
	void grab();
private:
	RoboCompCamera::TCamParams params;
	gazebo::Client *client;
	gazebo::SimulationIface *simIface;
	gazebo::CameraIface *camIface;
	std::vector<uint8_t> *imgVectors;
	int *sizes;
	IppiSize imageSize;
	IppiRect imageRoi;
};

#endif
#endif


