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
#     v4l2capture.cpp - Implementation for v4l2capture.h                    #
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
#include "v4l2capture.h"


V4l2capture::V4l2capture()
{
	_finished=false;
}


V4l2capture::~V4l2capture()
{
	for (int i=0; i<MAX_CAMERAS; ++i)
	{
		if ( vd[i].isstreaming )
		{
			int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			ioctl ( vd[i].fd, VIDIOC_STREAMOFF, &type );
			vd[i].isstreaming = 0;
		}

		free(vd[i].framebuffer);
		vd[i].framebuffer = NULL;
		free(vd[i].videodevice);
		free(vd[i].status);
		vd[i].videodevice = NULL;
		vd[i].status = NULL;
	}
}


/* return >= 0 ok otherwhise -1 */
int isv4l2Control(struct vdIn *vd, int control, struct v4l2_queryctrl *queryctrl)
{
	int err =0;
	queryctrl->id = control;
	err = ioctl(vd->fd, VIDIOC_QUERYCTRL, queryctrl);
	if (err < 0)
	{
		perror("ioctl querycontrol error");
	}
	else if (queryctrl->flags & V4L2_CTRL_FLAG_DISABLED)
	{
		printf("control %s disabled\n", (char *) queryctrl->name);
	}
	else if (queryctrl->flags & V4L2_CTRL_TYPE_BOOLEAN)
	{
		return 1;
	}
	else if (queryctrl->type & V4L2_CTRL_TYPE_INTEGER)
	{
		return 0;
	}
	else {
		printf("contol %s unsupported\n", (char *) queryctrl->name);
		return 0;
	}
	return -1;
}

int enum_frame_intervals(int dev, __u32 pixfmt, __u32 width, __u32 height)
{
	int ret;
	struct v4l2_frmivalenum fival;

	memset(&fival, 0, sizeof(fival));
	fival.index = 0;
	fival.pixel_format = pixfmt;
	fival.width = width;
	fival.height = height;
	printf("\tTime interval between frame: ");
	while ((ret = ioctl(dev, VIDIOC_ENUM_FRAMEINTERVALS, &fival)) == 0) {
		if (fival.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
				printf("%u/%u, ",
						fival.discrete.numerator, fival.discrete.denominator);
		} else if (fival.type == V4L2_FRMIVAL_TYPE_CONTINUOUS) {
				printf("{min { %u/%u } .. max { %u/%u } }, ",
						fival.stepwise.min.numerator, fival.stepwise.min.numerator,
						fival.stepwise.max.denominator, fival.stepwise.max.denominator);
				break;
		} else if (fival.type == V4L2_FRMIVAL_TYPE_STEPWISE) {
				printf("{min { %u/%u } .. max { %u/%u } / "
						"stepsize { %u/%u } }, ",
						fival.stepwise.min.numerator, fival.stepwise.min.denominator,
						fival.stepwise.max.numerator, fival.stepwise.max.denominator,
						fival.stepwise.step.numerator, fival.stepwise.step.denominator);
				break;
		}
		fival.index++;
	}
	printf("\n");
	if (ret != 0 && errno != EINVAL) {
		printf("ERROR enumerating frame intervals: %d\n", errno);
		return errno;
	}

	return 0;
}
int enum_frame_sizes(int dev, __u32 pixfmt)
{
	int ret;
	struct v4l2_frmsizeenum fsize;

	memset(&fsize, 0, sizeof(fsize));
	fsize.index = 0;
	fsize.pixel_format = pixfmt;
	while ((ret = ioctl(dev, VIDIOC_ENUM_FRAMESIZES, &fsize)) == 0) {
		if (fsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
			printf("{ discrete: width = %u, height = %u }\n",
					fsize.discrete.width, fsize.discrete.height);
			ret = enum_frame_intervals(dev, pixfmt,
					fsize.discrete.width, fsize.discrete.height);
			if (ret != 0)
				printf("  Unable to enumerate frame sizes.\n");
		} else if (fsize.type == V4L2_FRMSIZE_TYPE_CONTINUOUS) {
			printf("{ continuous: min { width = %u, height = %u } .. "
					"max { width = %u, height = %u } }\n",
					fsize.stepwise.min_width, fsize.stepwise.min_height,
					fsize.stepwise.max_width, fsize.stepwise.max_height);
			printf("  Refusing to enumerate frame intervals.\n");
			break;
		} else if (fsize.type == V4L2_FRMSIZE_TYPE_STEPWISE) {
			printf("{ stepwise: min { width = %u, height = %u } .. "
					"max { width = %u, height = %u } / "
					"stepsize { width = %u, height = %u } }\n",
					fsize.stepwise.min_width, fsize.stepwise.min_height,
					fsize.stepwise.max_width, fsize.stepwise.max_height,
					fsize.stepwise.step_width, fsize.stepwise.step_height);
			printf("  Refusing to enumerate frame intervals.\n");
			break;
		}
		fsize.index++;
	}
	if (ret != 0 && errno != EINVAL) {
		printf("ERROR enumerating frame sizes: %d\n", errno);
		return errno;
	}

	return 0;
}

bool V4l2capture::init(RoboCompCamera::TCamParams &params_, RoboCompJointMotor::JointMotorPrx head_, RoboCompDifferentialRobot::DifferentialRobotPrx base_)
{
	qDebug() << "Initiating V4l2Capture...";
	params = params_;

	head=head_;
	base=base_;

	/// We shold be checking params before!!!
	if (params.device.c_str() == NULL ) qFatal ( "V4L2Capture::init() fatal error: device is NULL" );
	if (params.width == 0 || params.height == 0 )
	{
		qFatal ("V4L2Capture::init() fatal error: Frame size is 0");
		return -1;
	}
	int grabmethod = 1; // mmap
	int format;
	if (params.mode=="MODE_640x480_YUV422" || params.mode=="MODE_320x240_YUV422")
		format = V4L2_PIX_FMT_YUYV;
	else if (params.mode=="MODE_640x480_GREY" || params.mode=="MODE_640x480_MONO" || params.mode=="MODE_320x240_GREY" || params.mode=="MODE_320x240_MONO")
		format = V4L2_PIX_FMT_GREY;
	else
	{
		qDebug() << params.mode.c_str();
		qFatal("No valid format");
		return -1;
	}

	QString deviceString = QString::fromStdString(params.device.c_str());
	QStringList deviceList = deviceString.split(",", QString::SkipEmptyParts);

	numCameras = deviceList.size();
	qDebug() << numCameras<<deviceString;
	for (int i=0; i<numCameras; ++i)
	{
		qDebug() << "Configuring camera:" << i;
		memset(&vd[i], 0, sizeof(struct vdIn));

		deviceList[i].remove(" ");
		vd[i].videodevice = (char *)calloc(deviceList[i].size()+1, sizeof(char));
		vd[i].status = (char *)calloc(1, 100*sizeof(char));
		//Esto debería hacerse con una expresión regular
		snprintf(vd[i].videodevice, 12, "%s", deviceList[i].toStdString().c_str());
		printf ( "V4L2Capture::init() info: current video device %s \n", vd[i].videodevice );
		vd[i].width = params.width;
		vd[i].height = params.height;
		vd[i].formatIn = format;
		vd[i].grabmethod = grabmethod;
		vd[i].fps = params.FPS;

		if ((vd[i].fd = open(vd[i].videodevice, O_RDWR)) == -1 )
			qFatal ( "V4L2Capture::init() fatal error: Opening V4L interface" );

		memset(&vd[i].cap, 0, sizeof(struct v4l2_capability));

		if (ioctl(vd[i].fd, VIDIOC_QUERYCAP, &vd[i].cap) < 0)
			qFatal ( "V4L2Capture::init() fatal error: Unable to query device %s ", vd[i].videodevice );

		if ((vd[i].cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)==0)
			qFatal ( "V4L2Capture::init() fatal error: video capture not supported %s ", vd[i].videodevice );

		if (vd[i].grabmethod)
		{
			if (!(vd[i].cap.capabilities & V4L2_CAP_STREAMING))
				qFatal ( "V4L2Capture::init() fatal error: %s does not support streaming", vd[i].videodevice );
		}
		else
		{
			if (!(vd[i].cap.capabilities & V4L2_CAP_READWRITE) )
				qFatal ( "V4L2Capture::init() fatal error: %s does not support read i/o ", vd[i].videodevice );
		}

		struct v4l2_fmtdesc fmt;
		memset(&fmt, 0, sizeof(fmt));
		fmt.index = 0;
		fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

		while (ioctl(vd[i].fd, VIDIOC_ENUM_FMT, &fmt) == 0)
		{
			fmt.index++;
			printf("{ pixelformat = '%c%c%c%c', description = '%s' }\n",fmt.pixelformat & 0xFF, (fmt.pixelformat >> 8) & 0xFF, 	(fmt.pixelformat >> 16) & 0xFF, (fmt.pixelformat >> 24) & 0xFF,	fmt.description);
			if (enum_frame_sizes(vd[i].fd, fmt.pixelformat) != 0)
				printf("  Unable to enumerate frame sizes.\n");
		}

		// set format in
		memset (&vd[i].fmt, 0, sizeof(struct v4l2_format));
		vd[i].fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		vd[i].fmt.fmt.pix.width = vd[i].width;
		vd[i].fmt.fmt.pix.height = vd[i].height;
		vd[i].fmt.fmt.pix.pixelformat = vd[i].formatIn;
		vd[i].fmt.fmt.pix.field = V4L2_FIELD_ANY;

		if (ioctl(vd[i].fd, VIDIOC_S_FMT, &vd[i].fmt) < 0)
			qFatal ( "V4L2Capture::init() fatal error: Unable to set format through VIDIOC_S_FMT" );

		if ( (vd[i].fmt.fmt.pix.width!=(uint)vd[i].width) || (vd[i].fmt.fmt.pix.height!=(uint)vd[i].height) )
			qFatal ( "V4L2Capture::init() fatal error: Size %dx%d is not available. Suggested %dx%d", vd[i].width, vd[i].height, vd[i].fmt.fmt.pix.width, vd[i].fmt.fmt.pix.height);

		/* set framerate */
//		struct v4l2_streamparm* setfps;
//		setfps=(struct v4l2_streamparm *) calloc(1, sizeof(struct v4l2_streamparm));
//		memset(setfps, 0, sizeof(struct v4l2_streamparm));
//		setfps->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
//		setfps->parm.capture.timeperframe.numerator=1;
//		setfps->parm.capture.timeperframe.denominator=30;
//		if( ioctl(vd[i].fd, VIDIOC_S_PARM, setfps) < 0 )
//				qFatal ( "V4L2Capture::init() fatal error: Unable to set frame rate through VIDIOC_S_PARM" );

		// request buffers
		memset(&vd[i].rb, 0, sizeof(struct v4l2_requestbuffers));
		vd[i].rb.count = NB_BUFFER;
		vd[i].rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		vd[i].rb.memory = V4L2_MEMORY_MMAP;
		if (ioctl(vd[i].fd, VIDIOC_REQBUFS, &vd[i].rb) < 0)
			qFatal ( "V4L2Capture::init() fatal error: Unable to allocate buffers through VIDIOC_REQBUFS" );

		// map the buffers
		for (int ii=0; ii<NB_BUFFER; ii++)
		{
			memset(&vd[i].buf, 0, sizeof(struct v4l2_buffer));
			vd[i].buf.index = ii;
			vd[i].buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			vd[i].buf.memory = V4L2_MEMORY_MMAP;
			if (ioctl(vd[i].fd, VIDIOC_QUERYBUF, &vd[i].buf ) < 0)
				qFatal ( "V4L2Capture::init() fatal error: Unable to query buffer through VIDIOC_QUERYBUF. Error number %d ", errno );
			vd[i].mem[ii] = mmap(0, vd[i].buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, vd[i].fd, vd[i].buf.m.offset );
			if (vd[i].mem[ii] == MAP_FAILED)
				qFatal ( "V4L2Capture::init() fatal error: Unable to map the buffer through mmap. Error number: %d ",errno );
		}

		// Queue the buffers.
		for (int ii = 0; ii < NB_BUFFER; ++ii)
		{
			memset(&vd[i].buf, 0, sizeof(struct v4l2_buffer));
			vd[i].buf.index = ii;
			vd[i].buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			vd[i].buf.memory = V4L2_MEMORY_MMAP;

			if (ioctl(vd[i].fd, VIDIOC_QBUF, &vd[i].buf) < 0)
				qFatal ( "V4L2Capture::init() fatal error: Unable to queue buffers through VIDIOC_QBUF. Error number: %d",errno );
		}

		// alloc a temp buffer to reconstruct the pict
		vd[i].framesizeIn = ( vd[i].width * vd[i].height << 1 );
		switch ( vd[i].formatIn )
		{
		case V4L2_PIX_FMT_MJPEG:
				qFatal("MJPEG not supported");
// 			if ((vd[i].tmpbuffer=(unsigned char *)calloc(1, (size_t)vd[i].framesizeIn)) == 0)
// 				qFatal ( "V4L2Capture::init() fatal error: not enough memory to allocate vd[i].tmpbuffer" );
// 			vd[i].framebuffer = (unsigned char *)calloc(1, (size_t)vd[i].width*(vd[i].height + 8)*2);
			break;
		case V4L2_PIX_FMT_YUYV:
			vd[i].framebuffer = (unsigned char *)calloc(1, ( size_t ) vd[i].framesizeIn);
			break;
		case V4L2_PIX_FMT_GREY:
			vd[i].framebuffer = (unsigned char *)calloc(1, ( size_t ) vd[i].framesizeIn);
			break;
		default:
			printf("V4LCapture::init() V4L2 error: Format not recognized!!\n");
			return false;
			break;
		}

		if (vd[i].framebuffer == 0)
			qFatal ( "V4L2Capture::init() fatal error: not enough memory to allocate vd[i].framebuffer" );

		int control;
		struct v4l2_control control_s;
		struct v4l2_queryctrl queryctrl;
		int err;

		// Set the power line frequency value
		control = V4L2_CID_POWER_LINE_FREQUENCY;
		control_s.id = control;
		if (isv4l2Control(&vd[i], control, &queryctrl) < 0) return false;
		if (params.lineFreq == 60) control_s.value =  V4L2_CID_POWER_LINE_FREQUENCY_60HZ;
		else control_s.value =  V4L2_CID_POWER_LINE_FREQUENCY_50HZ;
		if ((err = ioctl(vd[i].fd, VIDIOC_S_CTRL, &control_s)) < 0)
			qFatal("V4LCapture::init() V4L2 error: ioctl set_light_frequency_filter error\n");
		printf("CameraComp: Power line frequency: %dHz\n", params.lineFreq);

		// Set the saturation value
		control = V4L2_CID_SATURATION;
		control_s.id = control;
		if (isv4l2Control(&vd[i], control, &queryctrl) < 0) return false;
		control_s.value = params.saturation;
		if ((err = ioctl(vd[i].fd, VIDIOC_S_CTRL, &control_s)) < 0)
			qFatal("V4LCapture::init() V4L2 error: ioctl set_saturation error\n");
		printf("CameraComp: Saturation: %d\n", params.saturation);


	// Set the auto exposure off for max speed
		control = V4L2_CID_EXPOSURE_AUTO;
		control_s.id = control;
//		if (isv4l2Control(&vd[i], control, &queryctrl) < 0) return false;
//		control_s.value = 8;
// 		if ((err = ioctl(vd[i].fd, VIDIOC_S_CTRL, &control_s)) < 0)
// 			qFatal("V4LCapture::init() V4L2 error: ioctl set_exposureOff error\n");
// 		printf("CameraComp: AutoExposure: %d\n", 1);
//
// 	// Set the auto exposure off for max speed
 //		control = V4L2_CID_EXPOSURE_ABSOLUTE;
 //		control_s.id = control;
 //		if (isv4l2Control(&vd[i], control, &queryctrl) < 0) return false;
 //		control_s.value = 200;
 //		if ((err = ioctl(vd[i].fd, VIDIOC_S_CTRL, &control_s)) < 0)
 //			qFatal("V4LCapture::init() V4L2 error: ioctl set_exposureOff error\n");
 //		printf("CameraComp: AutoExposure: %d\n", 900);


}


	return true;
}


#define HEADERFRAME1 0xaf
void V4l2capture::uvcGrab(  )
{
	for (int i=0; i<numCameras; ++i)
	{
		if (vd[i].isstreaming == false)
		{
			int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			if (ioctl(vd[i].fd, VIDIOC_STREAMON, &type) < 0)
				qFatal ( "V4l2capture fatal error: Unable to start capture using VIDIOC_STREAMON: Error number: %d", errno );
			else
				vd[i].isstreaming = 1;
		}

		memset(&vd[i].buf, 0, sizeof(struct v4l2_buffer));
		vd[i].buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		vd[i].buf.memory = V4L2_MEMORY_MMAP;

		//* Get kinematic information from base and head to attach to frame
		if (params.talkToBase==true)
		{
			try { base->getBaseState(bState); }
			catch (const Ice::Exception& ex) { qDebug() << "CameraComp in V4L2 - Error reading BaseComp state"; }
		}
		if (params.talkToJointMotor==true)
		{
			try {
				RoboCompJointMotor::MotorStateMap map;
				RoboCompCommonHead::dmotorsState aux;
				head->getAllMotorState(map);
				
				aux["leftPan"] = map["leftPan"];
				aux["rightPan"] = map["rightPan"];
				aux["tilt"] = map["tilt"];
				aux["neck"] = map["neck"];
				
				hState.motorsState = aux;
				hState.isMoving = aux["neck"].isMoving or aux["tilt"].isMoving or aux["leftPan"].isMoving or aux["rightPan"].isMoving;
			}
			catch (const Ice::Exception& ex) { qDebug() << "CameraComp in V4l2 - Error reading JointMotor state"; }
		}

		if (ioctl(vd[i].fd, VIDIOC_DQBUF, &vd[i].buf) < 0)
			qFatal("V4l2capture fatal error: Unable to dequeue buffer using VIDIOC_DQBUF: Error number: %d", errno);

		mu->lock();
		if (vd[i].buf.bytesused > (uint)vd[i].framesizeIn)
			memcpy(vd[i].framebuffer, vd[i].mem[vd[i].buf.index], (size_t) vd[i].framesizeIn);
		else
			memcpy(vd[i].framebuffer, vd[i].mem[vd[i].buf.index], (size_t) vd[i].buf.bytesused);
		mu->unlock();

		if (ioctl(vd[i].fd, VIDIOC_QBUF, &vd[i].buf) < 0)
			qFatal ( "V4l2capture fatal error: Unable to requeue buffer using VIDIOC_QBUF: Error number: %d", errno );
	}
}


///* Run thread method

void V4l2capture::run()
{
	while (!_finished)
	{
		printFPS();
		uvcGrab();
	}
	if (_finished)
	{
		printf("Finished thread\n");
	}
}

///
///Accessors for public interface
///

void V4l2capture::getYUVPtr(uchar *buf, uchar cam, RoboCompCommonHead::THeadState& hState_, RoboCompDifferentialRobot::TBaseState& bState_)
{
	hState_ = hState;
	bState_ = bState;
	mu->lock();
	  memcpy (buf, vd[cam].framebuffer, params.size*2 );
	mu->unlock();
}

void V4l2capture::getYRGBPtr(uchar *dest, uchar cam, RoboCompCommonHead::THeadState&,RoboCompDifferentialRobot::TBaseState&)
{
	IppiSize isize = {params.width, params.height};
	Ipp8u aux[params.size*3];
	Ipp8u *planos[3] = {dest+params.size, dest+ ( params.size*2 ), dest + ( 3*params.size ) };

	mu->lock();
	  for (int j=0; j<params.size; j++)
		  dest[j] = vd[cam].framebuffer[j*2];
	  ippiYUV422ToRGB_8u_C2C3R(vd[cam].framebuffer, params.width*2, aux, params.width*3, isize);
	mu->unlock();
	ippiCopy_8u_C3P3R(aux, params.width*3, planos, params.width, isize);
}

void V4l2capture::getYPtr(uchar *dest, uchar cam, RoboCompCommonHead::THeadState &hState_, RoboCompDifferentialRobot::TBaseState &bState_)
{
	hState_ = hState;
	bState_ = bState;
	mu->lock();
	for (int j=0; j<params.size; j++)
		dest[j] = vd[cam].framebuffer[j*2];
	mu->unlock();
}

void V4l2capture::getYLogPolarPtr(uchar *dest, uchar cam, RoboCompCommonHead::THeadState& hState_, RoboCompDifferentialRobot::TBaseState& bState_)
{
	uchar buf[params.size],buf1[params.height*params.height]; //!!Se puede usar dest como buf intermedio
	IppiSize roi = {params.height, params.height};
	RoboCompCommonHead::THeadState dummy;
	RoboCompDifferentialRobot::TBaseState dummy2;

	hState_ = hState;
	bState_ = bState;
	getYPtr (buf, cam, dummy, dummy2 );

	//Extract square image height*height
	ippiCopy_8u_C1R ( buf, params.width, buf1, params.height, roi );
	lp->convertPromedio ( buf1,dest );
}

void V4l2capture::getRGBPackedPtr ( uchar *dest, uchar cam, RoboCompCommonHead::THeadState& hState_, RoboCompDifferentialRobot::TBaseState& bState_)
{
	IppiSize isize;

	isize.width = params.width;
	isize.height = params.height;

	mu->lock();
	  hState_ = hState;
	  bState_ = bState;
	  ippiYUV422ToRGB_8u_C2C3R (vd[cam].framebuffer, isize.width*2, dest, isize.width*3 ,isize);
	mu->unlock();
}

