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
#include "ieee1394capture.h"
#include <iostream>

ieee1394capture::ieee1394capture()
{
	_finished=false;
	sleeped=false;
	CREATED_BUS = CREATED_CAMS = false;
}

ieee1394capture::~ieee1394capture()
{
	cleanup();
}

void ieee1394capture::cleanup()
{
	int32_t i;
	if (CREATED_CAMS == true)
	{
		for (i = 0; i < numCameras; i++) {
			dc1394_video_set_transmission(cameras[i], DC1394_OFF);
			dc1394_capture_stop(cameras[i]);
		}
		CREATED_CAMS=false;
	}
	if ( CREATED_BUS == true )
	{
		dc1394_free(d);
		CREATED_BUS=false;
	}
}


bool ieee1394capture::init(RoboCompCamera::TCamParams &params_, RoboCompJointMotor::JointMotorPrx head_ , RoboCompDifferentialRobot::DifferentialRobotPrx base_ )
{
	params = params_;
	head = head_;
	base = base_;

	int32_t i;

	fps = (dc1394framerate_t)params.FPS;//15;
	res = (dc1394video_mode_t)0;

	switch (fps) {
	case 1: fps =  DC1394_FRAMERATE_1_875; break;
	case 3: fps =  DC1394_FRAMERATE_3_75; break;
	case 15: fps = DC1394_FRAMERATE_15; break;
	case 30: fps = DC1394_FRAMERATE_30; break;
	case 60: fps = DC1394_FRAMERATE_60; break;
	default: fps = DC1394_FRAMERATE_7_5; break;
	}

	switch (res) {
	case 1:
		res = DC1394_VIDEO_MODE_640x480_YUV411;
		device_width = 640;
		device_height = 480;
		break;
	case 2:
		res = DC1394_VIDEO_MODE_640x480_RGB8;
		device_width = 640;
		device_height = 480;
		break;
	default:
		res = DC1394_VIDEO_MODE_320x240_YUV422;
		device_width = 320;
		device_height = 240;
		break;
	}


	/// Get handle
	qDebug() << "ieee1394capture::init() -> Initializating first Firewire Card in the system...";
	if (!(d = dc1394_new()))
	{
		qDebug() << "ieee1394capture::init() -> Fatal error: Unable to aquire a handle to the Ieee1394 device";
		qDebug() << "Please check if the kernel modules `ieee1394',`raw1394' and `ohci1394' are loaded or if you have read/write access to /dev/raw1394 and to /dev/video1394-0 " ;
		return false;
	}
	CREATED_BUS = true;

	/// Create camera interfaces
	numCameras = 0;
	err = dc1394_camera_enumerate(d, &list);
	DC1394_ERR_RTN(err, "Failed to enumerate cameras");
	if (list->num == 0)
	{
		dc1394_log_error("No cameras found");
		return 1;
	}
	numCameras = 0;
	for (uint32_t  di = 0; di < list->num; di++)
	{
		if (numCameras >= MAX_CAMERAS)
			break;
		cameras[numCameras] = dc1394_camera_new(d, list->ids[di].guid);
		if (!cameras[numCameras])
		{
			dc1394_log_warning("Failed to initialize camera with guid %llx", list->ids[di].guid);
			continue;
		}
		printf("Camera #%d\n", numCameras);
		numCameras++;
	}
	dc1394_camera_free_list(list);
	if ( numCameras < 1 )
	{
		qDebug() << "ieee1394capture::init() -> Fatal error: No cameras found in the bus! Called from ";
		cleanup();
		return false;
	}
	/// Check if one camera has become the root node
/*	for ( int n=0; n < numCameras; n++ )
	{
		if ( cameraNodeList[n] == numNodes )
		{
			qDebug() << "ieee1394capture::init() -> Fatal error: Sorry, your camera is the highest numbered node of the bus, and has therefore become the root node." ;
			cleanup();
			return false;
		}
	}*/
	CREATED_CAMS = true;

	/// Setup cameras for capture
	qDebug() << "ieee1394capture::init() -> Searching cameras with requested parameters:";
	printf("%s\n",params.mode.c_str());
	if (params.mode == "MODE_320x240_YUV422")
	{
		res = DC1394_VIDEO_MODE_320x240_YUV422;
		params.width = 320;
		params.height = 240;
	}
	else if (params.mode == "MODE_640x480_YUV422" )
	{
		res = DC1394_VIDEO_MODE_640x480_YUV422;
		params.width = 640;
		params.height = 480;
	}
	else if (params.mode == "MODE_640x480_RGB" )
	{
		res = DC1394_VIDEO_MODE_640x480_RGB8;
		params.width = 640;
		params.height = 480;
	}
	else if (params.mode == "MODE_640x480_YUV411")
	{
		res = DC1394_VIDEO_MODE_640x480_YUV411;
		params.width = 640;
		params.height = 480;
	}
	else if (params.mode == "MODE_640x480_MONO")
	{
		res = DC1394_VIDEO_MODE_640x480_MONO8;
		params.width = 640;
		params.height = 480;
	}
	else if (params.mode == "MODE_640x480_MONO16")
	{
		res = DC1394_VIDEO_MODE_640x480_MONO16;
		params.width = 640;
		params.height = 480;
	}
	else if (params.mode == "MODE_516x338_YUV422")
	{
		res = DC1394_VIDEO_MODE_FORMAT7_1;
		params.width = 516;
		params.height = 338;
	}
	else qFatal("ieee1394capture::init() -> Image Mode %s not available. Aborting...", params.mode.c_str());
	params.size = params.width*params.height;
	if (params.FPS!=15 and params.FPS!=30)
	{
		qWarning("ieee1394capture::init() -> Framerate %d not available. Aborting...", params.FPS );
		cleanup();
		return false;
	}

	dc1394format7modeset_t info;

	for (i = 0; i < numCameras; i++)
	{

	    if (params.mode == "MODE_516x338_YUV422")
	    {
		err = dc1394_format7_get_modeset(cameras[i], &info);

		for( int j=0;j<DC1394_VIDEO_MODE_FORMAT7_NUM;j++)
		{
		  qDebug() << info.mode[j].present;
		  qDebug() << info.mode[j].size_x;
		  qDebug() << info.mode[j].size_y;
		  qDebug() << info.mode[j].max_size_x;
		  qDebug() << info.mode[j].max_size_y;

		  qDebug() << info.mode[j].pos_x;
		  qDebug() << info.mode[j].pos_y;

		  qDebug() << info.mode[j].unit_size_x;
		  qDebug() << info.mode[j].unit_size_y;
		  qDebug() << info.mode[j].unit_pos_x;
		  qDebug() << info.mode[j].unit_pos_y;

		  qDebug() << info.mode[j].pixnum;

		  qDebug() << info.mode[j].packet_size; /* in bytes */
		  qDebug() << info.mode[j].unit_packet_size;
		  qDebug() << info.mode[j].max_packet_size;
		}
	    }


   	    release_iso_and_bw(i);

	    err = dc1394_video_set_mode(cameras[i], res);
	    DC1394_ERR_CLN_RTN(err, cleanup(), "Could not set video mode");

	    err = dc1394_video_set_iso_speed(cameras[i], DC1394_ISO_SPEED_400);
	    DC1394_ERR_CLN_RTN(err, cleanup(), "Could not set ISO speed");


	    //For format 7 modes only
	    if (params.mode == "MODE_516x338_YUV422")
	    {
	   //  uint32_t packet_size;

	    //  err=dc1394_format7_set_image_size(cameras[i], res, 514, 384);
	    //  DC1394_ERR_RTN(err,"Could not set image size");
	    //  err=dc1394_format7_get_recommended_packet_size(cameras[i], res, &packet_size);
	    //  DC1394_ERR_RTN(err,"Could not get format 7 recommended packet size");
	    //  err=dc1394_format7_set_roi(cameras[i], res, DC1394_COLOR_CODING_YUV422, packet_size, 0,0, 514, 384);
	    //  DC1394_ERR_RTN(err,"Could not set ROI");
	      qDebug() << "ya";
	    }

	    err = dc1394_video_set_framerate(cameras[i], fps);
	    DC1394_ERR_CLN_RTN(err, cleanup(), "Could not set framerate");

	    err = dc1394_capture_setup(cameras[i], NUM_BUFFERS, DC1394_CAPTURE_FLAGS_DEFAULT);
	    DC1394_ERR_CLN_RTN(err, cleanup(), "Could not setup camera-\nmake sure that the video mode and framerate are\nsupported by your camera");

	    err = dc1394_video_set_transmission(cameras[i], DC1394_ON);
	    DC1394_ERR_CLN_RTN(err, cleanup(), "Could not start camera iso transmission");


	}
	fflush(stdout);
	qDebug() << "	ieee1394capture::init() -> Iso transmission started.";


	///Buffers de imagen
	qDebug() << "BUFFERS DE IMAGEN ------------------------------------------";
	for ( int i=0; i < numCameras; i++ )
	{
		AimgBuffer[i] = ( Ipp8u * ) ippsMalloc_8u ( params.size * 9 );
		BimgBuffer[i] = ( Ipp8u * ) ippsMalloc_8u ( params.size * 9 );
		img8u_lum[i] = AimgBuffer[i];
		img8u_YUV[i] = AimgBuffer[i]+params.size;
		localYRGBImgBufferPtr[i] = BimgBuffer[i];
		qDebug() << "Reservando" << params.size * 9 <<" para localYRGBImgBufferPtr["<<i<<"]";
		printf("(de %p a %p)\n", localYRGBImgBufferPtr[i], localYRGBImgBufferPtr[i]+(params.size*9-1));
	}


	planos[0]=BimgBuffer[0]+params.size*3;
	planos[1]=BimgBuffer[0]+ ( params.size*4 );
	planos[2]=BimgBuffer[0]+ ( params.size*5 );
	//img8u_aux = BimgBuffer[0]+(params.size*6);

	imgSize_ipp.width=params.width;
	imgSize_ipp.height=params.height;

	return true;

}

void ieee1394capture::release_iso_and_bw(int i)
{
    uint32_t val;
    if ( dc1394_video_get_bandwidth_usage(cameras[i], &val) == DC1394_SUCCESS &&
         dc1394_iso_release_bandwidth(cameras[i], val) == DC1394_SUCCESS )
        qDebug() << "Succesfully released " << val << " bytes of Bandwidth.";
    if ( dc1394_video_get_iso_channel(cameras[i], &val) == DC1394_SUCCESS &&
         dc1394_iso_release_channel(cameras[i], val) == DC1394_SUCCESS )
        qDebug() << "Succesfully released ISO channel #" << val << "." ;
}


//  Capturador imagen
bool ieee1394capture::grab( )
{
	uchar *capBuff;
	int i,j;
	static bool A=true;
	uchar *p8u_lum, *p8u_YUV;
// 	QTime time;
// qDebug()<<"enter grab";
// 	time.start();
	if (params.talkToJointMotor==true)
	{
//   		qDebug()<<"peticion de estado de la cabeza";
		try
		{
// 			qDebug()<<"antes1";
			RoboCompJointMotor::MotorStateMap map;
			RoboCompCommonHead::dmotorsState aux;
			head->getAllMotorState(map);
			
			aux["leftPan"] = map["leftPan"] ;
			aux["rightPan"] = map["rightPan"];
			aux["tilt"] = map["tilt"];
			aux["neck"] = map["neck"];
			
			hStateBefore.motorsState = aux;
			hStateBefore.isMoving = aux["neck"].isMoving or aux["tilt"].isMoving or aux["leftPan"].isMoving or aux["rightPan"].isMoving;
// 			qDebug()<<"despues1";
		}
		catch (const Ice::Exception& ex)
		{
			qDebug() << "CameraComp - Error reading JointMotor state";
			return false;
		}
	}
	if (params.talkToBase==true)
	{
//   		qDebug()<<"peticion de estado de la base";
		try
		{
			base->getBaseState (bStateBefore);
		}
		catch (const Ice::Exception& ex)
		{
			qDebug() << "BaseComp - Error reading Base state";
			return false;
		}
	}

// 	qDebug()<<"tiempo transcurrido desde la petición de estado"<<time.elapsed();
//   	qDebug()<<"captura de imagenes";

	for (i=0; i<numCameras; i++)
	{
//                   qDebug()<<"lectura camara"<<i;
		if (dc1394_capture_dequeue(cameras[i], DC1394_CAPTURE_POLICY_WAIT, &frames[i]) != DC1394_SUCCESS)
		{

			qWarning("ieee1394capture::grab() -> Fatal error. Called from ... ");
			return false;
		}

//                  qDebug()<<"fin lectura camara"<<i;
		capBuff = (uchar*)(frames[i]->image);

/*		if ( res == MODE_640x480_MONO )
		{
			memcpy ( img8u_lum[i], capBuff, params.size );   //Copia del buffer de captura en 8 bits por pixels
		}
		else*/
		{
			p8u_lum = img8u_lum[i];
			p8u_YUV = img8u_YUV[i];
			for (j=0; j< params.size; j++)
			{
				p8u_lum[j] = p8u_YUV[j*2] = capBuff[j*2+1];
				p8u_YUV[j*2+1] = capBuff[j*2];
			}
		}

		//Double buffering
// 		for (i = 0; i < numCameras; i++)
// 		{
			if (frames[i]) dc1394_capture_enqueue(cameras[i], frames[i]);
// 		}
	}

// 	time.restart();
// 	qDebug()<<"segunda peticion de estado de la cabeza";
	if (params.talkToJointMotor==true)
	{
		try
		{
// 			qDebug()<<"antes2";
			RoboCompJointMotor::MotorStateMap map;
			RoboCompCommonHead::dmotorsState aux;
			head->getAllMotorState(map);
			
			aux["leftPan"] = map["leftPan"];
			aux["rightPan"] = map["rightPan"];
			aux["tilt"] = map["tilt"];
			aux["neck"] = map["neck"];
			
			hStateAfter.motorsState = aux;
			hStateAfter.isMoving = aux["neck"].isMoving or aux["tilt"].isMoving or aux["leftPan"].isMoving or aux["rightPan"].isMoving;
			
// 			qDebug()<<"despuess2";
		}
		catch (const Ice::Exception& ex)
		{
			qDebug() << "CameraComp - Error reading JointMotor state";
			return false;
		}
	}
	if (params.talkToBase==true)
	{
		try
		{
			base->getBaseState (bStateAfter);
		}
		catch (const Ice::Exception& ex)
		{
			qDebug() << "BaseComp - Error reading Base state";
			return false;
		}
	}

// 	qDebug()<<"tiempo transcurrido desde la segunda petición de estado"<<time.elapsed();

//          qDebug()<<"actualización del doble buffer";
	mu->lock();
	for (i=0; i < numCameras; i++)
	{
		if (A)
		{
			localYRGBImgBufferPtr[i] = AimgBuffer[i];
			img8u_lum[i] = BimgBuffer[i];
			img8u_YUV[i] = BimgBuffer[i] + params.size;
		}
		else
		{
			localYRGBImgBufferPtr[i] = BimgBuffer[i];
			img8u_lum[i] = AimgBuffer[i];
			img8u_YUV[i] = AimgBuffer[i] + params.size;
		}
	}
	hState=hStateAfter;

// 	for(RoboCompCommonHead::dmotorsState::const_iterator it = hStateAfter.motorsState.begin(); it != hStateAfter.motorsState.end(); ++it)
// 	{
// 		if (it->second.pos != hStateBefore.motorsState[it->first].pos)
// 		{
// 			hState.motorsState[it->first].isMoving=true;
// // 			hState.isMoving=true;
// 		}
// 
// 	}

	bState=bStateAfter;
	A = !A;
	mu->unlock();


//  	qDebug()<<"grab done";

	return true;
}

/**
 * Run method for thread
 */
void ieee1394capture::run( )
{
	while ( !_finished )
	{
// 		if(!sleeped)
		{
			if ( grab() == false )
				qFatal ( "ieee1394capture::run() -> Fatal error: grab() returned error" );
			printFPS();

		}
	}
}


/// Interface methods

void ieee1394capture::getYUVPtr(uchar *buf, uchar cam, RoboCompCommonHead::THeadState &hState_ , RoboCompDifferentialRobot::TBaseState &bState_)
{
	mu->lock();
	memcpy ( buf, localYRGBImgBufferPtr[cam] + params.size, params.size*2 );
	hState_ = hState;
	bState_ = bState;
	mu->unlock();
}

void ieee1394capture::getYPtr(uchar *buf, uchar cam, RoboCompCommonHead::THeadState &hState_ , RoboCompDifferentialRobot::TBaseState &bState_)
{
	QMutexLocker locker ( mu );

	if (params.numCams > numCameras )
	{
		qFatal( "ieee1394capture::getYUVPtr() -> Fatal error. Not enough cameras in the bus");
		cleanup();
	}
	if ( ( cam == params.leftCamera )  or ( cam == params.rightCamera ))
	{
		memcpy ( buf, localYRGBImgBufferPtr[cam], params.size );
		hState_ = hState;
		bState_ = bState;
	}
	if (  cam == params.bothCameras )
	{
		memcpy ( buf, localYRGBImgBufferPtr[params.leftCamera], params.size );
		memcpy ( buf + params.size, localYRGBImgBufferPtr[params.rightCamera], params.size );
		hState_ = hState;
		bState_ = bState;
	}
}

void ieee1394capture::getYLogPolarPtr ( uchar *buf, uchar cam, RoboCompCommonHead::THeadState& hState,RoboCompDifferentialRobot::TBaseState& bState)
{
	RoboCompCommonHead::THeadState dummy;
	RoboCompDifferentialRobot::TBaseState dummy2;

	uchar buf1[params.height* params.height];

	getYPtr ( buf1, cam, dummy, dummy2 );
	lp->convert ( buf1,buf );

	hState = hState;
	bState = bState;
}

void ieee1394capture::getRGBPackedPtr ( uchar *buf, uchar cam, RoboCompCommonHead::THeadState& hState_ ,RoboCompDifferentialRobot::TBaseState& bState_)
{
	IppiSize imgSize_ipp = { params.width, params.height};

	if (params.numCams > numCameras )
	{
		qFatal( "ieee1394capture::getYUVPtr() -> Fatal error. Not enough cameras in the bus");
		cleanup();
	}
	if ( ( cam == params.leftCamera )  or ( cam == params.rightCamera ))
	{
		mu->lock();
		ippiYUV422ToRGB_8u_C2C3R ( localYRGBImgBufferPtr[cam]+params.size, params.width*2, buf, params.width*3, imgSize_ipp);
		mu->unlock();
	}

	if (  cam == params.bothCameras )
	{
		mu->lock();
		ippiYUV422ToRGB_8u_C2C3R(localYRGBImgBufferPtr[params.leftCamera]+params.size,  params.width*2, buf,               params.width*3, imgSize_ipp );
		ippiYUV422ToRGB_8u_C2C3R(localYRGBImgBufferPtr[params.rightCamera]+params.size, params.width*2, buf+params.size*3, params.width*3, imgSize_ipp );
		mu->unlock();
	}


	hState_ = hState;
	bState_ = bState;
}

void ieee1394capture::getYRGBPtr (uchar *buf, uchar cam, RoboCompCommonHead::THeadState& hState_, RoboCompDifferentialRobot::TBaseState& bState_)
{
	IppiSize imgSize_ipp = { params.width, params.height};
	Ipp8u *planos[3];
	Ipp8u img8u_aux[params.size*3];

	if (params.numCams > numCameras )
	{
		qFatal( "ieee1394capture::getYUVPtr() -> Fatal error. Not enough cameras in the bus");
		cleanup();
	}
	mu->lock();

	memcpy ( buf, localYRGBImgBufferPtr[cam], params.size );
	ippiYUV422ToRGB_8u_C2C3R ( localYRGBImgBufferPtr[cam]+params.size,params.width*2,img8u_aux,params.width*3,imgSize_ipp );
	planos[0] = buf + params.size;
	planos[1] = buf + 2*params.size;
	planos[2] = buf + 3*params.size;
	ippiCopy_8u_C3P3R ( img8u_aux,params.width * 3,planos, params.width,imgSize_ipp );
	mu->unlock();

	hState_ = hState;
	bState_ = bState;
}

