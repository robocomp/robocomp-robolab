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

#include "prosilicacapture.h"

// callback called when the camera is plugged/unplugged
void static CameraEventCB(void* Context, tPvInterface Interface, tPvLinkEvent Event, unsigned long UniqueId)
{
	switch(Event)
	{
		case ePvLinkAdd:
		{
			printf("camera %lu plugged\n",UniqueId);
			break;
		}
		case ePvLinkRemove:
		{
			printf("camera %lu unplugged\n",UniqueId);
			break;
		}
		default:
			break;
	}
}

ProsilicaCapture::ProsilicaCapture()
{
  _finished=false;
}

ProsilicaCapture::~ProsilicaCapture()
{
	PvLinkCallbackUnRegister(CameraEventCB,ePvLinkAdd);
	PvLinkCallbackUnRegister(CameraEventCB,ePvLinkRemove);

	// uninitialise the API
	PvUnInitialize();
}

bool ProsilicaCapture::init(RoboCompCamera::TCamParams & params_, RoboCompJointMotor::JointMotorPrx head_ , RoboCompDifferentialRobot::DifferentialRobotPrx base_ )
{
	params = params_;
	head = head_;
	base = base_;

	GCamera = new tCamera[params.numCams];

	// initialise the Prosilica API
	if (!PvInitialize())
	{
		memset(GCamera, 0, sizeof(tCamera)*params.numCams);

		PvLinkCallbackRegister(CameraEventCB,ePvLinkAdd,NULL);
		PvLinkCallbackRegister(CameraEventCB,ePvLinkRemove,NULL);

		// wait for a camera to be plugged
		WaitForCamera();
		if (numCameras < params.numCams)
			return false;
		else
			params.numCams = numCameras;
		// grab a camera from the list
		if (CameraGrab() && !GCamera[0].Abort)
		{
			printf("%s: %d\n", __FILE__, __LINE__);
			// setup the camera
			if (CameraSetup())
			{
				for (int i=0; i<params.numCams; i++)
				{
					PvAttrUint32Set(GCamera[i].Handle, "Width", params.width);
					PvAttrUint32Set(GCamera[i].Handle, "Height", params.height);
					PvAttrEnumSet(GCamera[i].Handle, "PixelFormat", "Yuv422");
					PvAttrEnumSet(GCamera[i].Handle, "FrameStartTriggerMode", "FixedRate");
					PvAttrFloat32Set(GCamera[i].Handle, "FrameRate", float(params.FPS));

					tPvUint32 attr;
					qDebug()<< "Camera number " << i;
					PvAttrUint32Get(GCamera[i].Handle, "Width", &attr);
					qDebug() << "Camera Frame Width set to: " << attr;
					PvAttrUint32Get(GCamera[i].Handle, "Height", &attr);
					qDebug() << "Camera Frame 	Height set to: " << attr;
					tPvFloat32 attrf;
					PvAttrFloat32Get(GCamera[i].Handle, "FrameRate", &attrf);
					qDebug() << "Camera Frame FPS set to: " << attrf;
					char f[10];
					PvAttrEnumGet(GCamera[i].Handle, "PixelFormat", f, 10,NULL);
					qDebug() << "Image format set to: " << f;
				}

				// start streaming from the camera
				if(CameraStart())
				{
					//tPvAttrListPtr pListPtr;
					//long unsigned int pLength;
					//PvAttrList(GCamera.Handle, &pListPtr, &pLength);
	// 				for(int i=0;i<pLength;i++)
	// 				{
	// 				  PvAttrUint32Get(GCamera.Handle, pListPtr[i], &attr);
	//  				  qDebug()<< QString(pListPtr[i])+ " " + QString::number(attr);
	// 				}


					qDebug() << "ProsilicaCapture::init() - Camara streaming now ...";
					///Buffers de imagen
					qDebug() << "BUFFERS DE IMAGEN ------------------------------------------";
					for ( int i=0; i < numCameras; i++ )
					{
						AimgBuffer[i] = ( Ipp8u * ) ippsMalloc_8u ( params.size * 9 );
						BimgBuffer[i] = ( Ipp8u * ) ippsMalloc_8u ( params.size * 9 );
						img8u_lum[i] = AimgBuffer[i];
						img8u_YUV[i] = &(AimgBuffer[i][params.size]);
						localYRGBImgBufferPtr[i] = BimgBuffer[i];
						qDebug() << "Reservando" << params.size * 9 <<" para localYRGBImgBufferPtr["<<i<<"]";
					}

					planos[0]=BimgBuffer[0]+params.size*3;
					planos[1]=BimgBuffer[0]+ ( params.size*4 );
					planos[2]=BimgBuffer[0]+ ( params.size*5 );
					//img8u_aux = BimgBuffer[0]+(params.size*6);

					imgSize_ipp.width=params.width;
					imgSize_ipp.height=params.height;
					return true;
				}
				else
				{
					printf("ProsilicaCapture::init() - failed to start streaming\n");
	// 			CameraUnsetup();
				}
			}
			else
			{
				printf("ProsilicaCapture::init() - failed to setup the camera\n");
			}
		}
		else
		{
			printf("ProsilicaCapture::init() - failed to find a camera\n");
		}
	}
	else
	{
		printf("ProsilicaCapture::init() - failed to initialise the API\n");
	}

	return false;
}

void ProsilicaCapture::cleanup()
{
}

bool ProsilicaCapture::grab()
{
	uchar *capBuff;
	int i,j;
	static bool A=true;
	uchar *p8u_lum, *p8u_YUV;

	if (params.talkToJointMotor==true)
	{
		try
		{
			RoboCompJointMotor::MotorStateMap map;
			head->getAllMotorState(map);
			hStateBefore.motorsState = map;
			hStateBefore.isMoving = map["neck"].isMoving or map["tilt"].isMoving or map["leftPan"].isMoving or map["rightPan"].isMoving;
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
			base->getBaseState (bStateBefore);
		}
		catch (const Ice::Exception& ex)
		{
			qDebug() << "BaseComp - Error reading Base state";
			return false;
		}
	}

	for (i=0; i<params.numCams; i++)
	{
		tPvErr err;
		// we tell it to capture
		err = PvCaptureQueueFrame(GCamera[i].Handle,&(GCamera[i].Frame), NULL);
		if (err == ePvErrUnplugged)
		{
			printf("camera was unplugged\n");
			return 0;
		}
		else if (err == ePvErrBadSequence)
		{
			printf("bad sequence\n");
			return 1;
		}
		else if (err == ePvErrQueueFull)
			printf("full\n");
		
		if (err == ePvErrSuccess or err == ePvErrQueueFull)
		{
			//and wait until done
			if (PvCaptureWaitForFrameDone(GCamera[i].Handle,&(GCamera[i].Frame), PVINFINITE) != 0)
			{
				printf("PvCaptureWaitForFrameDone error\n");
				return false;
			}
		}
		capBuff = (uchar*)(GCamera[i].Frame.ImageBuffer);
		p8u_lum = img8u_lum[i];
		p8u_YUV = img8u_YUV[i];
		for (j=0; j< params.size; j++)
		{
		  p8u_lum[j] = p8u_YUV[j*2] = capBuff[j*2+1];
		  p8u_YUV[j*2+1] = capBuff[j*2];
		}
	}
	mu->lock();
	for (i=0; i<params.numCams; i++)
	{
		if (A)
		{
			localYRGBImgBufferPtr[i] = AimgBuffer[i];
			img8u_lum[i] = BimgBuffer[i];
			img8u_YUV[i] = &(BimgBuffer[i][params.size]);
		}
		else
		{
			localYRGBImgBufferPtr[i] = BimgBuffer[i];
			img8u_lum[i] = AimgBuffer[i];
			img8u_YUV[i] = &(AimgBuffer[i][params.size]);
		}
	}
	hState=hStateBefore;
	bState=bStateBefore;
	A = !A;
	mu->unlock();

	return true;
}

void ProsilicaCapture::run()
{
  while ( !_finished )
	{
	  if ( grab() == false )
		qFatal ( "ProsilicaCapture::run() -> Fatal error: grab() returned error" );
	   printFPS();
	}
}

///////////////////
//// Driver specific methods
///////////////////

void  ProsilicaCapture::Sleep(unsigned int time)
{
    struct timespec t,r;

    t.tv_sec    = time / 1000;
    t.tv_nsec   = (time % 1000) * 1000000;

    while(nanosleep(&t,&r)==-1)
        t = r;
}

// wait for a camera to be plugged
void ProsilicaCapture::WaitForCamera()
{
	static QTime lapse = QTime::currentTime();
    printf("waiting for a camera ...\n");
    while (PvCameraCount()==0 && GCamera[0].Abort==0 && lapse.elapsed()<5000)
	{
	  Sleep(250);
	}
	numCameras = PvCameraCount();
	qDebug()<< "Num cameras found: " << numCameras;
}

// get the first camera found
bool ProsilicaCapture::CameraGrab()
{
	tPvUint32 count,connected;
	tPvCameraInfo list[numCameras];

	count = PvCameraList(list, numCameras,  &connected);
	if (count == (uint)numCameras)
	{
		for (int i=0; i<numCameras;i++)
		{
		  GCamera[i].UID = list[i].UniqueId;
		  printf("grabbing camera %s %ld\n",list[i].SerialString, GCamera[i].UID);
		}
		return true;
	}
	return false;
}

// open the camera
bool ProsilicaCapture::CameraSetup()
{
	for (int i=0; i<numCameras; i++)
	{
		printf("%s: %d\n", __FILE__, __LINE__);
		printf("%ld\n", GCamera[i].UID);
		tPvErr errCode = PvCameraOpen(GCamera[i].UID, ePvAccessMaster, &(GCamera[i].Handle));
		if (errCode == ePvErrSuccess)
		{
			qDebug() << "CameraSetUp Ok" << i;
		}
		else if (errCode == ePvErrAccessDenied)
		{
			qDebug() << "CameraSetUp Failed: Access denied" << i;
		}
		else if (errCode == ePvErrNotFound)
		{
			qDebug() << "CameraSetUp Failed: Camera not found" << i;
		}
		else
		{
			qDebug() << "CameraSetUp Failed: unknown error" << i;
			return false;
		}
	}
	return true;
}

// setup and start streaming
bool ProsilicaCapture::CameraStart()
{
    unsigned long FrameSize = 0;
	for(int i=0;i<numCameras;i++)
	{
	  // Auto adjust the packet size to max supported by the network, up to a max of 8228.
	  PvCaptureAdjustPacketSize(GCamera[i].Handle,8228);

	  // how big should the frame buffers be?
	  if(PvAttrUint32Get(GCamera[i].Handle,"TotalBytesPerFrame",&FrameSize) != 0)
		qFatal("1");
// 		return false;

	  qDebug()<<"camera"<<i<<"frameSize"<<FrameSize;

	  // allocate the buffer for each frames
	  GCamera[i].Frame.ImageBuffer = new char[FrameSize];
		if(GCamera[i].Frame.ImageBuffer)
			  GCamera[i].Frame.ImageBufferSize = FrameSize;
		else
		  qFatal("2");
// 		  return false;

	  //set the camera in acquisition mode
	  if(PvCaptureStart(GCamera[i].Handle)!= 0)
		qFatal("3");
// 		return false;
	  //set the acquisition mode to continuous
//	  PvAttrEnumSet(GCamera[i].Handle, "AcquisitionMode", "Continuous");

	  if(PvCommandRun(GCamera[i].Handle,"AcquisitionStart") != 0)
	  {
// 		if that fail, we reset the camera to non capture mode
		PvCaptureEnd(GCamera[i].Handle) ;
		qFatal("4");
// 		return false;
	  }
	}
	return true;
}

// stop streaming
void ProsilicaCapture::CameraStop()
{
	for(int i=0;i<numCameras;i++)
	{
	  printf("stopping streaming\n");
	  PvCommandRun(GCamera[i].Handle,"AcquisitionStop");
	  PvCaptureEnd(GCamera[i].Handle);
	}
}

// unsetup the camera
void ProsilicaCapture::CameraUnsetup()
{
  for(int i=0;i<numCameras;i++)
	{
	  printf("clearing the queue\n");
	  // dequeue all the frame still queued (this will block until they all have been dequeued)
	  PvCaptureQueueClear(GCamera[i].Handle);
	  // then close the camera
	  printf("and closing the camera\n");
	  PvCameraClose(GCamera[i].Handle);

	  // delete all the allocated buffers
	  //     for(int i=0;i<FRAMESCOUNT;i++)
	  //         delete [] (char*)GCamera.Frames[i].ImageBuffer;

	  GCamera[i].Handle = NULL;
	}
}

///////////////////////
//// Service methods
///////////////////////

void ProsilicaCapture::getYUVPtr(uchar *buf, uchar cam, RoboCompCommonHead::THeadState &hState_, RoboCompDifferentialRobot::TBaseState &bState_)
{
  mu->lock();
	memcpy ( buf, localYRGBImgBufferPtr[cam] + params.size, params.size*2 );
	hState_ = hState;
	bState_ = bState;
  mu->unlock();
}

void ProsilicaCapture::getYPtr(uchar *buf, uchar cam, RoboCompCommonHead::THeadState &hState_ , RoboCompDifferentialRobot::TBaseState &bState_ )
{

  if (params.numCams > numCameras and params.numCams>0)
  {
	  qFatal( "ieee1394capture::getYUVPtr() -> Fatal error. Not enough cameras in the bus");
	  cleanup();
  }
  mu->lock();
	  memcpy ( buf, localYRGBImgBufferPtr[cam], params.size );
	  hState_ = hState;
	  bState_ = bState;
  mu->unlock();

}

void ProsilicaCapture::getYRGBPtr(uchar *buf, uchar cam, RoboCompCommonHead::THeadState & hState_ ,RoboCompDifferentialRobot::TBaseState & bState_)
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

void ProsilicaCapture::getYLogPolarPtr(uchar *buf, uchar cam, RoboCompCommonHead::THeadState & hState_, RoboCompDifferentialRobot::TBaseState & bState_)
{
  RoboCompCommonHead::THeadState dummy;
  RoboCompDifferentialRobot::TBaseState dummy2;

  uchar buf1[params.height* params.height];

  getYPtr ( buf1, cam, dummy, dummy2 );
  lp->convert ( buf1,buf );

  hState = hState;
  bState = bState;
}

void ProsilicaCapture::getRGBPackedPtr(uchar *buf, uchar cam, RoboCompCommonHead::THeadState & hState_, RoboCompDifferentialRobot::TBaseState &bState_)
{
	IppiSize imgSize_ipp = { params.width, params.height};

	if (params.numCams > numCameras and  params.numCams >0)
	{
		qFatal( "ieee1394capture::getYUVPtr() -> Fatal error. Not enough cameras in the bus");
		cleanup();
	}
	mu->lock();
	  ippiYUV422ToRGB_8u_C2C3R ( localYRGBImgBufferPtr[cam]+params.size, params.width*2, buf, params.width*3, imgSize_ipp);
	mu->unlock();

	hState_ = hState;
	bState_ = bState;
}
#endif

