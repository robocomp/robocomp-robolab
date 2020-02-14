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
#include "worker.h"

// callback called when the camera is plugged/unplugged
void static CameraEventCB(void* Context,tPvInterface Interface,tPvLinkEvent Event,unsigned long UniqueId)
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

/**
* \brief Default constructor
*/
Worker::Worker(RoboCompDifferentialRobot::DifferentialRobotPrx differentialrobotprx, RoboCompJointMotor::JointMotorPrx jointmotorprx)
{
#ifdef USE_IPP
	ippSetNumThreads(1);
#endif
	
	base = differentialrobotprx;
	head = jointmotorprx;


	mu = new QMutex();

	_finished=false;
	Period = BASIC_PERIOD*1000;
	
	
	printf("Worker::Worker\n");
}

/**
* \brief Default destructor
*/
Worker::~Worker()
{
	PvLinkCallbackUnRegister(CameraEventCB,ePvLinkAdd);
	PvLinkCallbackUnRegister(CameraEventCB,ePvLinkRemove);

	// uninitialise the API
	PvUnInitialize();
}
///Common Behavior

void Worker::killYourSelf()
{
	rDebug("Killing myself");
	emit kill();
	exit(1);
}
/**
* \brief Change compute period
* @param per Period in ms
*/
void Worker::setPeriod(int p)
{
	rDebug("Period changed"+QString::number(p));
	Period = p*1000;
}
/**
* \brief
* @param params_ Parameter list received from monitor thread
*/
bool Worker::setParams(RoboCompCommonBehavior::ParameterList _params)
{
	printf("Worker::setParams\n");
	busparams.mode = RoboCompCameraBus::YUV422Mode;
	busparams.numCameras = QString::fromStdString(_params["Prosilica.NumCameras"].value).toInt();
	///check numcameras menor que MAX_CAMERAS
	if (busparams.numCameras < 1 or busparams.numCameras > MAX_CAMERAS)
		return false;
	busparams.FPS = QString::fromStdString(_params["Prosilica.FPS"].value).toInt();
	busparams.width = QString::fromStdString(_params["Prosilica.Width"].value).toInt();
	busparams.height = QString::fromStdString(_params["Prosilica.Height"].value).toInt();
	busparams.talkToBase = QString::fromStdString(_params["Prosilica.TalkToBase"].value).contains("true");;
	QStringList deviceList = QString::fromStdString(_params["Prosilica.Device"].value).split(",", QString::SkipEmptyParts);

	cameraFormat.width = busparams.width;
	cameraFormat.height = busparams.height;
	cameraFormat.modeImage = RoboCompCameraBus::YUV422;
	cameraFormat.size = cameraFormat.width * cameraFormat.height;
	
	RoboCompCameraBus::CameraParams camera;
	cameraList.resize(busparams.numCameras);
	cameraParamsList.resize(busparams.numCameras);
	busparams.devices.resize(busparams.numCameras);
	for (int i=0; i<busparams.numCameras; i++)
	{
		busparams.devices[i] = deviceList.at(i).toStdString();
		std::string s = QString::number(i).toStdString();
		camera.name = _params["Prosilica.Params_" + s +".name"].value; 
		camera.busId = QString::fromStdString(_params["Prosilica.Params_"+ s +".busId"].value).toShort();
		camera.invertedH = QString::fromStdString(_params["Prosilica.Params_"+ s +".invertedH"].value).contains("true");
		camera.invertedV = QString::fromStdString(_params["Prosilica.Params_"+ s +".invertedV"].value).contains("true");
		camera.focalX = QString::fromStdString(_params["Prosilica.Params_"+ s +".focalX"].value).toInt();
		camera.focalY = QString::fromStdString(_params["Prosilica.Params_"+ s +".focalY"].value).toInt();
		camera.saturation = QString::fromStdString(_params["Prosilica.Params_"+ s +".saturation"].value).toInt();
		camera.lineFreq = QString::fromStdString(_params["Prosilica.Params_"+ s +".linefreq"].value).toInt();
		cameraList[i] = camera.name;
		cameraParamsList[i] = camera;
	}

	//Reserve space for buffering
	for(int i=0;i<busparams.numCameras;i++)
	{
		imgBufferGrabA[i] = (uchar*)malloc(MAX_WIDTH*MAX_HEIGHT*3);
		imgBufferGrabB[i] = (uchar*)malloc(MAX_WIDTH*MAX_HEIGHT*3);
		imgBufferTransformA[i] = (uchar*)malloc(MAX_WIDTH*MAX_HEIGHT*3);
		imgBufferTransformB[i] = (uchar*)malloc(MAX_WIDTH*MAX_HEIGHT*3);
		imgBufferGrabPtr[i] = imgBufferTransformB[i];
	}

	init();

	this->start();
	return true;
}


/**
* \brief Thread method
*/
void Worker::run()
{
	forever
	{
// 		rDebug("worker running");
		usleep(Period);
	}
}
/**
* \brief Init method 
*/
bool Worker::init()
{
	printf("Worker::init\n");
	rDebug("Initiating ProsilicaCapture...");
	GCamera = new tCamera[busparams.numCameras];
	
	// initialise the Prosilica API
	if (!PvInitialize())
	{
		memset(GCamera, 0, sizeof(tCamera)*busparams.numCameras);

		PvLinkCallbackRegister(CameraEventCB,ePvLinkAdd,NULL);
		PvLinkCallbackRegister(CameraEventCB,ePvLinkRemove,NULL);

		// wait for a camera to be plugged
		if (WaitForCamera() < busparams.numCameras) 
			return false;
		
		// grab a camera from the list
		if(CameraGrab() && !GCamera[0].Abort)
		{
			// setup the camera
			if(CameraSetup())
			{
				for(int i=0;i<busparams.numCameras;i++)
				{
					PvAttrUint32Set(GCamera[i].Handle, "Width", busparams.width);
					PvAttrUint32Set(GCamera[i].Handle, "Height", busparams.height);
					PvAttrEnumSet(GCamera[i].Handle, "PixelFormat", "Yuv422");
					PvAttrEnumSet(GCamera[i].Handle, "FrameStartTriggerMode", "FixedRate");
					PvAttrFloat32Set(GCamera[i].Handle, "FrameRate", 15.f);

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
					qDebug() << "ProsilicaCapture::init() - Camara streaming now ...";
					return true;
				}
				else
					printf("ProsilicaCapture::init() - failed to start streaming\n");
			}
		}
		else
			printf("ProsilicaCapture::init() - failed to find a camera\n");
	}
	else
		printf("ProsilicaCapture::init() - failed to initialise the API\n");
	
	return false;
}

bool Worker::grab()
{
	uchar *capBuff;
	int i,j;
	static bool A=true;
	uchar *p8u_lum, *p8u_YUV;
	
	if (busparams.talkToBase==true)
	{
		try
		{
			head->getAllMotorState(map);
			hStateBefore = map;
		}
		catch (const Ice::Exception& ex)
		{
			rDebug("Error reading JointMotor state");
			return false;
		}
		try
		{
			base->getBaseState (bStateBefore);
		}
		catch (const Ice::Exception& ex)
		{
			rDebug("Error reading Base state");
			return false;
		}
	}

	for (i=0; i<busparams.numCameras; i++)
	{
		// we tell it to capture
		if(PvCaptureQueueFrame(GCamera[i].Handle,&(GCamera[i].Frame),NULL) != 0)
			qFatal("1");
		else
		{ //and wait until done
			if (PvCaptureWaitForFrameDone(GCamera[i].Handle,&(GCamera[i].Frame),PVINFINITE) != 0)
				qFatal("2");
		}
		capBuff = (uchar*)(GCamera[i].Frame.ImageBuffer);
		
		p8u_lum = img8u_lum[i];
		p8u_YUV = img8u_YUV[i];
		for (j=0; j< cameraFormat.size; j++)
		{
			p8u_lum[j] = p8u_YUV[j*2] = capBuff[j*2+1];
			p8u_YUV[j*2+1] = capBuff[j*2];
		}
	}
	//* Get kinematic information from base and head to attach to frame
	if (busparams.talkToBase==true)
	{
		try
		{
			head->getAllMotorState(map);
			hStateAfter = map;
		}
		catch (const Ice::Exception& ex)
		{
			rDebug("Error reading JointMotor state");
			return false;
		}
		try{
			base->getBaseState (bStateAfter);
		}
		catch (const Ice::Exception& ex)
		{
			rDebug("Error reading Base state");
			return false;
		}
	}
	mu->lock();
		bState=bStateAfter;
		hState=hStateAfter;
		for(RoboCompJointMotor::MotorStateMap::const_iterator it = hStateAfter.begin(); it != hStateAfter.end(); ++it)
		{
			if (it->second.pos != hStateBefore[it->first].pos)
				hState[it->first].isMoving=true;
		}
		for(int i=0;i<busparams.numCameras;i++)
		{
			if(A)
			{
/*				localYRGBImgBufferPtr[i] = imgBuffer1[i];
				img8u_lum[i] = imgBuffer2[i];
				img8u_YUV[i] = imgBuffer2[i] + cameraFormat.size;*/
			}
			else
			{
/*				localYRGBImgBufferPtr[i] = imgBuffer2[i];
				img8u_lum[i] = imgBuffer1[i];
				img8u_YUV[i] = imgBuffer1[i] + cameraFormat.size;*/
			}
		}
		A = !A;
	mu->unlock();
	return true;
}

///
///Accessors for public interface
///
void Worker::getSyncImages(const RoboCompCameraBus::CameraList& _cameraList, const RoboCompCameraBus::Format& format, bool all,RoboCompCameraBus::ImageList& imagelist)
{
	
}

void Worker::getImage(const string &camera, const RoboCompCameraBus::Format &format, RoboCompCameraBus::Image &image)
{
	
}

/**
 * \brief Return Base position 
 * @return TBaseState Struct contains base position compute by odometry
*/
void Worker::getBaseState(RoboCompDifferentialRobot::TBaseState &bState_)
{
	mu->lock();
	bState_ = bState;
	mu->unlock();	
}
/**
 * \brief Return Head position 
 * @return THeadState Struct contains head position
*/
void Worker::getHeadState (RoboCompJointMotor::MotorStateMap &hState_)
{
	mu->lock();
	hState_ = hState;
	mu->unlock();
}



/// PROSILICA SDK
void Worker::Sleep(unsigned int time)
{
	struct timespec t,r;

	t.tv_sec   = time / 1000;
	t.tv_nsec   = (time % 1000) * 1000000;

	while(nanosleep(&t,&r)==-1)
		t = r;
}

// wait for a camera to be plugged
int Worker::WaitForCamera()
{
	static QTime lapse = QTime::currentTime();
	int n = 0;
	printf("waiting for a camera ...\n");
	while(PvCameraCount()==0 && GCamera[0].Abort==0 && lapse.elapsed()<5000)
	{
		Sleep(250);
	}
	n = PvCameraCount();
	rDebug("Num cameras found: "+QString::number(n));
	return n;
}

// get the first camera found
bool Worker::CameraGrab()
{
	tPvUint32 count,connected;
	tPvCameraInfo list[busparams.numCameras];

	count = PvCameraList(list,busparams.numCameras,&connected);
	if (count == (uint)busparams.numCameras)
	{
		for (int i=0; i<busparams.numCameras;i++)
		{
			GCamera[i].UID = list[i].UniqueId;
			printf("grabbing camera %s\n",list[i].SerialString);
		}
		return true;
	}
	else
		return false;
}

// open the camera
bool Worker::CameraSetup()
{
	for(int i=0; i<busparams.numCameras;i++)
	{
		if( PvCameraOpen(GCamera[i].UID,ePvAccessMaster,&(GCamera[i].Handle)) != ePvErrSuccess)
		{
			qDebug() << "CameraSetUp Fail" << i;
			return false;
		}
		else
			qDebug() << "CameraSetUp Ok" << i;
	}
   return true;
}

// setup and start streaming
bool Worker::CameraStart()
{
    unsigned long FrameSize = 0;

	for(int i=0;i<busparams.numCameras;i++)
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
void Worker::CameraStop()
{
	for(int i=0;i<busparams.numCameras;i++)
	{
		printf("stopping streaming\n");
		PvCommandRun(GCamera[i].Handle,"AcquisitionStop");
		PvCaptureEnd(GCamera[i].Handle);
	}
}

// unsetup the camera
void Worker::CameraUnsetup()
{
	for(int i=0;i<busparams.numCameras;i++)
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
