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
#include "dummycapture.h"


DummyCapture::DummyCapture()
{
	_finished = false;
}

void DummyCapture::setImage(unsigned char *image, int _size)
{
	if (size < _size) {
		unsigned char *t = AimgBuffer[0];
		AimgBuffer[0] = new unsigned char[_size];
		size = _size;
		free(t);
	}
	memcpy(AimgBuffer[0], image, _size);
}

bool DummyCapture::init(RoboCompCamera::TCamParams &params_, RoboCompJointMotor::JointMotorPrx head_ , RoboCompDifferentialRobot::DifferentialRobotPrx base_)
{
	head = head_;
	base = base_;
	size=params_.size;
	cout << "Alloc: " << params_.size << endl;
	AimgBuffer[0] = (Ipp8u *) ippsMalloc_8u(params_.size * 9);
	BimgBuffer[0] = (Ipp8u *) ippsMalloc_8u(params_.size * 9);
	img8u_lum[0] = AimgBuffer[0];
	img8u_YUV[0] = AimgBuffer[0] + params_.size;
	localYRGBImgBufferPtr[0] = BimgBuffer[0];

	planos[0] = BimgBuffer[0] + params_.size*3;
	planos[1] = BimgBuffer[0] + (params_.size*4);
	planos[2] = BimgBuffer[0] + (params_.size*5);

	imgSize_ipp.width=params_.width;
	imgSize_ipp.height=params_.height;


/*
	QImage *initial = new QImage();
	if (initial->load("input.png")) {
		cout << "Loading input.png" << endl;
		unsigned char *ptr = AimgBuffer[0];
		QRgb rgb;
		for (int yvv = 0; yvv < params_.height; yvv++) {
			for (int xvv = 0; xvv < params_.width; xvv++) {
				rgb = initial->pixel(xvv, yvv);
				*(ptr++) = qRed(rgb);
				*(ptr++) = qGreen(rgb);
				*(ptr++) = qBlue(rgb);
			}
		}
	}
	size = params_.size * 3;
*/
/*
	memset(AimgBuffer[0], 255, params_.size*9);

	FILE *fd = fopen("input.raw", "r");
	size = 0;
	if (fd != NULL) {
		cout << "Loading input.raw" << endl;
		size = 0;
		unsigned char *ptr = AimgBuffer[0];
		while (!feof(fd)) {
			if (fread(ptr++, 1, 1, fd) != 1) break;
			++size;
		}
	} else {
		cout << "Not loading input.raw" << endl;
	}
*/

	return true;
}

/// Interface methods

void DummyCapture::getYUVPtr(uchar *buf, uchar cam, RoboCompCommonHead::THeadState &hState_, RoboCompDifferentialRobot::TBaseState &bState_)
{
	cam = cam;
	hState_ = hState_;
	bState_ = bState_;
	memcpy(buf, AimgBuffer[0], size);
}

void DummyCapture::getYPtr( uchar *buf, uchar cam, RoboCompCommonHead::THeadState &hState_, RoboCompDifferentialRobot::TBaseState &bState_)
{
	cam = cam;
	hState = hState_;
	bState = bState_;
/*
	IppiSize roi;
	if (size == 640*480*3) {
		roi.width=640;
		roi.height=480;
	} else {
		roi.width=320;
		roi.height=240;
	}
	ippiRGBToGray_8u_C3C1R(AimgBuffer[0], roi.width*3, buf, roi.width, roi);
*/
	memcpy(buf, AimgBuffer[0], size);
}

void DummyCapture::getYLogPolarPtr( uchar *buf, uchar cam, RoboCompCommonHead::THeadState &hState_, RoboCompDifferentialRobot::TBaseState &bState_)
{
	cam = cam;
	hState = hState_;
	bState = bState_;
	memcpy(buf, AimgBuffer[0], size);
}

void DummyCapture::getRGBPackedPtr(uchar *buf, uchar cam, RoboCompCommonHead::THeadState &hState_, RoboCompDifferentialRobot::TBaseState &bState_)
{
	cam = cam;
	hState = hState_;
	bState = bState_;
	memcpy(buf, AimgBuffer[0], size);
}
void DummyCapture::getYRGBPtr(uchar *buf, uchar cam, RoboCompCommonHead::THeadState &hState_, RoboCompDifferentialRobot::TBaseState &bState_)
{
	cam = cam;
	hState = hState_;
	bState = bState_;
	memcpy(buf, AimgBuffer[0], size);
}

/**
 * Run method for thread
 */
void DummyCapture::run( )
{
	while(!_finished)
	{
		grab();
		if ( params.talkToJointMotor==true )
			try {
				RoboCompJointMotor::MotorStateMap map;
				head->getAllMotorState(map);
				hState.motorsState = map;
				hState.isMoving = map["neck"].isMoving or map["tilt"].isMoving or map["leftPan"].isMoving or map["rightPan"].isMoving;
			}
			catch (const Ice::Exception& ex) { qDebug() << "CameraComp - Error reading JointMotor state"; }

		if ( params.talkToBase==true )
			try
			{	base->getBaseState (bState); }
			catch ( const Ice::Exception& ex ) { qDebug() << "BaseComp - Error reading Base state"; }

		usleep(1000000);
	}
}

