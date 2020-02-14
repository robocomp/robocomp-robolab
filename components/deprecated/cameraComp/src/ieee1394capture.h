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
#ifndef IEEE1394CAPTURE_H
#define IEEE1394CAPTURE_H

#include <stdlib.h>
#include <dc1394/dc1394.h>
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

#define MAX_PORTS   4
// #define MAX_CAMERAS 4
#define NUM_BUFFERS 10

class ieee1394capture : public Capturador
{
Q_OBJECT
public:
	ieee1394capture();
	~ieee1394capture();
	bool init(RoboCompCamera::TCamParams &params_, RoboCompJointMotor::JointMotorPrx head_ , RoboCompDifferentialRobot::DifferentialRobotPrx base_ );
	void cleanup();
	void run();
	void getYUVPtr(uchar*, uchar, RoboCompCommonHead::THeadState&,RoboCompDifferentialRobot::TBaseState &);
	void getYRGBPtr(uchar*, uchar, RoboCompCommonHead::THeadState& ,RoboCompDifferentialRobot::TBaseState&);
	void getYPtr(uchar*, uchar, RoboCompCommonHead::THeadState& , RoboCompDifferentialRobot::TBaseState &);
	void getYLogPolarPtr(uchar*, uchar, RoboCompCommonHead::THeadState&,RoboCompDifferentialRobot::TBaseState&);
	void getRGBPackedPtr(uchar*, uchar, RoboCompCommonHead::THeadState&,RoboCompDifferentialRobot::TBaseState&);

private:
	/* declarations for libdc1394 */
	int32_t numCameras;
	dc1394camera_t *cameras[MAX_CAMERAS];
	dc1394featureset_t features;
	dc1394video_frame_t *frames[MAX_CAMERAS];


	dc1394error_t err;
	dc1394_t *d;
	dc1394camera_list_t * list;
	unsigned long device_width, device_height;

	dc1394framerate_t fps;
	dc1394video_mode_t res;

	bool CREATED_BUS, CREATED_CAMS;

	int numNodes;
	dc1394video_mode_t auxModo;                          //*< Current mode

	Ipp8u* img8u_red[MAX_CAMERAS];
	bool leftIsGood, rightIsGood;

	void release_iso_and_bw(int i);

	/**
	 *    Grab method that maintains a buffer from the ieee1394 queue.
	 * @return true if a new frame is captured
	 */
	bool grab();
};

#endif
