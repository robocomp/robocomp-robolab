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

/****************************************************************************
#     v4l2capture.cpp - V4L2 Image grabber class.                           #
#                                                                           #
#     Copyright (C) 2005 2006 Laurent Pinchart &&  Michel Xhaard            #
#                                                                           #
# This program is free software; you can redistribute it and/or modify      #
# it under the terms of the GNU General Public License as published by      #
# the Free Software Foundation; either version 2 of the License, or         #
# (at your option) any later version.                                       #
#                                                                           #
# This program is distributed in the hope that it will be useful,           #
# but WITHOUT ANY WARRANTY; without even the implied warranty of            #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             #
# GNU General Public License for more details.                              #
#                                                                           #
# You should have received a copy of the GNU General Public License         #
# along with this program; if not, write to the Free Software               #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA #
#                                                                           #
****************************************************************************/
#ifndef V4L2CAPTURE_H
#define V4L2CAPTURE_H

#include <QtCore>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <string.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>

/* Fixed point arithmetic */
#define FIXED Sint32
#define FIXED_BITS 16
#define TO_FIXED(X) (((Sint32)(X))<<(FIXED_BITS))
#define FROM_FIXED(X) (((Sint32)(X))>>(FIXED_BITS))
#define NB_BUFFER 4
#define DHT_SIZE 432

#ifndef V4L2_CID_BACKLIGHT_COMPENSATION
#define V4L2_CID_BACKLIGHT_COMPENSATION	(V4L2_CID_PRIVATE_BASE+0)
#define V4L2_CID_POWER_LINE_FREQUENCY	(V4L2_CID_PRIVATE_BASE+1)
#define V4L2_CID_SHARPNESS		(V4L2_CID_PRIVATE_BASE+2)
#define V4L2_CID_HUE_AUTO		(V4L2_CID_PRIVATE_BASE+3)
#define V4L2_CID_FOCUS_AUTO		(V4L2_CID_PRIVATE_BASE+4)
#define V4L2_CID_FOCUS_ABSOLUTE		(V4L2_CID_PRIVATE_BASE+5)
#define V4L2_CID_FOCUS_RELATIVE		(V4L2_CID_PRIVATE_BASE+6)
#define V4L2_CID_EXPOSURE_AUTO		(V4L2_CID_PRIVATE_BASE+10)
#define V4L2_CID_EXPOSURE_ABSOLUTE		(V4L2_CID_PRIVATE_BASE+11)


#define V4L2_CID_POWER_LINE_FREQUENCY_DISABLED 0
#define V4L2_CID_POWER_LINE_FREQUENCY_50HZ 1
#define V4L2_CID_POWER_LINE_FREQUENCY_60HZ 2
#endif

#ifndef DEBUG
// #define DEBUG
#endif

#ifdef USE_IPP
#include <ipp.h>
#else
#include <ippWrapper.h>
#endif

#include <JointMotor.h>
#include <DifferentialRobot.h>
#include "capturador.h"

struct vdIn {
    int fd;
    char *videodevice;
    char *status;
    struct v4l2_capability cap;
    struct v4l2_format fmt;
    struct v4l2_buffer buf;
    struct v4l2_requestbuffers rb;
    void *mem[NB_BUFFER];
    unsigned char *framebuffer;
    int isstreaming;
    int grabmethod;
    int width;
    int height;
    int fps;
    int formatIn;
    int formatOut;
    int framesizeIn;
};

class V4l2capture : public Capturador
{
Q_OBJECT
public:
	V4l2capture();
	~V4l2capture();
	bool init(RoboCompCamera::TCamParams &params_, RoboCompJointMotor::JointMotorPrx, RoboCompDifferentialRobot::DifferentialRobotPrx);
	void run();  //Thread
	void getYUVPtr(uchar*, uchar cam, RoboCompCommonHead::THeadState&,RoboCompDifferentialRobot::TBaseState&);
	void getYRGBPtr(uchar* , uchar cam, RoboCompCommonHead::THeadState&,RoboCompDifferentialRobot::TBaseState&);
	void getYPtr(uchar* , uchar cam, RoboCompCommonHead::THeadState&, RoboCompDifferentialRobot::TBaseState &);
	void getYLogPolarPtr(uchar* , uchar cam, RoboCompCommonHead::THeadState&,RoboCompDifferentialRobot::TBaseState&);
	void getRGBPackedPtr(uchar* , uchar cam, RoboCompCommonHead::THeadState&,RoboCompDifferentialRobot::TBaseState&);
private:
	RoboCompCamera::TCamParams params;
	vdIn vd[MAX_CAMERAS];
	void uvcGrab();
};

#endif
