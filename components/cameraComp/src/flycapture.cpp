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
#include "flycapture.h"
#include <stdlib.h>
#include <ippdefs.h>


Flycapture::Flycapture()
{
}

Flycapture::~Flycapture()
{
   for ( unsigned int i = 0; i < numCameras; i++ )
    {
        ppCameras[i]->StopCapture();
        ppCameras[i]->Disconnect();
        delete ppCameras[i];
    }

    delete [] ppCameras;

}

void Flycapture::PrintBuildInfo()
{
    FC2Version fc2Version;
    Utilities::GetLibraryVersion( &fc2Version );
    char version[128];
    sprintf(
        version,
        "FlyCapture2 library version: %d.%d.%d.%d\n",
        fc2Version.major, fc2Version.minor, fc2Version.type, fc2Version.build );

    printf( "%s", version );

    char timeStamp[512];
    sprintf( timeStamp, "Application build date: %s %s\n\n", __DATE__, __TIME__ );

    printf( "%s", timeStamp );
}

void Flycapture::PrintCameraInfo( CameraInfo* pCamInfo )
{
    printf(
        "\n*** CAMERA INFORMATION ***\n"
        "Serial number - %u\n"
        "Camera model - %s\n"
        "Camera vendor - %s\n"
        "Sensor - %s\n"
        "Resolution - %s\n"
        "Firmware version - %s\n"
        "Firmware build time - %s\n\n",
        pCamInfo->serialNumber,
        pCamInfo->modelName,
        pCamInfo->vendorName,
        pCamInfo->sensorInfo,
        pCamInfo->sensorResolution,
        pCamInfo->firmwareVersion,
        pCamInfo->firmwareBuildTime );
}

void Flycapture::PrintFormat7Capabilities( Format7Info fmt7Info )
{
    printf(
        "Format 7 Capabilities\n"
		"Max image pixels: (%u, %u)\n"
        "Image Unit size: (%u, %u)\n"
        "Offset Unit size: (%u, %u)\n"
        "Pixel format bitfield: 0x%08x\n",
        fmt7Info.maxWidth,
        fmt7Info.maxHeight,
        fmt7Info.imageHStepSize,
        fmt7Info.imageVStepSize,
        fmt7Info.offsetHStepSize,
        fmt7Info.offsetVStepSize,
        fmt7Info.pixelFormatBitField );
}

void Flycapture::PrintError( Error error )
{
    error.PrintErrorTrace();
}

void Flycapture::cleanup()
{

  for (uint i=0; i<numCameras; i++)
  {
	error = ppCameras[i]->StopCapture();
	if (error != PGRERROR_OK)
	{
      PrintError( error );
	}
	// Disconnect the camera
	error = ppCameras[i]->Disconnect();
	if (error != PGRERROR_OK)
	{
	  PrintError( error );
	}
  }
}

bool Flycapture::grab()
{
	uchar *capBuff;
	uint i;
	int j;
	static bool A=true;
	uchar *p8u_lum, *p8u_YUV;


	if (params.talkToJointMotor==true)
	{
	  try
		{
			RoboCompJointMotor::MotorStateMap map;
			RoboCompCommonHead::dmotorsState aux;
			head->getAllMotorState(map);
			
			aux["leftPan"] = map["leftPan"];
			aux["rightPan"] = map["rightPan"];
			aux["tilt"] = map["tilt"];
			aux["neck"] = map["neck"];
			
			hStateBefore.motorsState = aux;
			hStateBefore.isMoving = aux["neck"].isMoving or aux["tilt"].isMoving or aux["leftPan"].isMoving or aux["rightPan"].isMoving;
		}
		catch (const Ice::Exception& ex)
		{
			qDebug() << "CameraComp - Error reading JointMotor state";
			std::cout << ex << endl;
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

	for (i=0; i<numCameras; i++)
	{
		error = ppCameras[i]->RetrieveBuffer( &image );
		if (error != PGRERROR_OK)
		{
			PrintError( error );
			return false;
		}
		capBuff = (uchar*)(image.GetData());
		p8u_lum = img8u_lum[i];
		p8u_YUV = img8u_YUV[i];
		
		for (j=0; j< this->params.size; j++)
		{
			p8u_lum[j] = p8u_YUV[j*2] = capBuff[j*2+1];
			p8u_YUV[j*2+1] = capBuff[j*2];
		}
	}
	if (params.talkToJointMotor==true)
	{
		try
		{
			RoboCompJointMotor::MotorStateMap map;
			RoboCompCommonHead::dmotorsState aux;
			head->getAllMotorState(map);
			
			aux["leftPan"] = map["leftPan"];
			aux["rightPan"] = map["rightPan"];
			aux["tilt"] = map["tilt"];
			aux["neck"] = map["neck"];
			
			hStateAfter.motorsState = aux;
			hStateAfter.isMoving = aux["neck"].isMoving or aux["tilt"].isMoving or aux["leftPan"].isMoving or aux["rightPan"].isMoving;
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

	for(RoboCompCommonHead::dmotorsState::const_iterator it = hStateAfter.motorsState.begin(); it != hStateAfter.motorsState.end(); ++it)
	{
		if(it->second.pos != hStateBefore.motorsState[it->first].pos)
		{
			hStateAfter.motorsState[it->first].isMoving=true;
			hStateAfter.isMoving=true;
		}

	}
	
	mu->lock();
	
	  for (i=0; i < numCameras; i++)
	  {
		  if (A)
		  {
			  localYRGBImgBufferPtr[i] = AimgBuffer[i];
			  img8u_lum[i] = BimgBuffer[i];
			  img8u_YUV[i] = BimgBuffer[i] + this->params.size;
		  }
		  else
		  {
			  localYRGBImgBufferPtr[i] = BimgBuffer[i];
			  img8u_lum[i] = AimgBuffer[i];
			  img8u_YUV[i] = AimgBuffer[i] + this->params.size;
		  }
	  }

	  hState=hStateAfter;
	  bState=bStateAfter;
	  A = !A;
	  
	mu->unlock();
	

	return true;

}

void Flycapture::run()
{
	while ( !_finished )
	{
		if ( grab() == false )
			qFatal ( "flycapture::run() -> Fatal error: grab() returned error" );
		printFPS();
	}
}

bool Flycapture::init(RoboCompCamera::TCamParams & params , RoboCompJointMotor::JointMotorPrx head_, RoboCompDifferentialRobot::DifferentialRobotPrx base_)
{
    fmt7Mode = MODE_1;
	fmt7PixFmt =  PIXEL_FORMAT_422YUV8;
	QProcess::execute("dc1394_reset_bus");

	this->params = params;
	head = head_;
	base = base_;

    PrintBuildInfo();

    error = busMgr.GetNumOfCameras((unsigned int *)&numCameras);
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return false;
    }

    printf( "Number of cameras detected: %u\n", numCameras );

    if ( numCameras < 1 )
    {
        printf( "Insufficient number of cameras... press Enter to exit.\n" );
        getchar();
        return false;
    }

    ppCameras = new Camera*[numCameras];

    // Connect to all detected cameras and attempt to set them to
    // a common video mode and frame rate
    for ( unsigned int i = 0; i < numCameras; i++)
    {
        ppCameras[i] = new Camera();

        PGRGuid guid;
        error = busMgr.GetCameraFromIndex( i, &guid );
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return false;
        }

        // Connect to a camera
        error = ppCameras[i]->Connect( &guid );
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return false;
        }

        // Get the camera information
        CameraInfo camInfo;
        error = ppCameras[i]->GetCameraInfo( &camInfo );
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return false;
        }

        PrintCameraInfo(&camInfo);

		 // Query for available Format 7 modes
	Format7Info fmt7Info;
	bool supported;
	fmt7Info.mode = fmt7Mode;
	error = ppCameras[i]->GetFormat7Info( &fmt7Info, &supported );
	std::cout << "Format 7-2 is supported " << supported << std::endl;
	if (error != PGRERROR_OK)
	{
		PrintError( error );
		return false;
	}

	PrintFormat7Capabilities( fmt7Info );

	if ( (fmt7PixFmt & fmt7Info.pixelFormatBitField) == 0 )
	{
		// Pixel format not supported!
		printf("Pixel format is not supported\n");
		return false;
	}
		
		
// 		Format7ImageSettings fmt7ImageSettings;
// 		fmt7ImageSettings.mode = fmt7Mode;
// 		fmt7ImageSettings.offsetX =  0;
// 		fmt7ImageSettings.offsetY =  0;
// 		fmt7ImageSettings.width = 516;
// 		fmt7ImageSettings.height = 388;
// 		fmt7ImageSettings.pixelFormat = PIXEL_FORMAT_422YUV8;//fmt7PixFmt;
// 
//		bool valid;
//		Format7PacketInfo fmt7PacketInfo;

// 		}

		// Set the settings to the camera

// 		error = ppCameras[i]->SetFormat7Configuration(&fmt7ImageSettings,50.0f);
// 		if (error != PGRERROR_OK)
// 		{
// 			PrintError( error );
// 			return false;
// 		}

	error = ppCameras[i]->SetVideoModeAndFrameRate( VIDEOMODE_320x240YUV422, FRAMERATE_30 );
	}

//
    error = Camera::StartSyncCapture( numCameras, (const Camera**)ppCameras );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        printf(
            "Error starting cameras."
            "This example requires cameras to be able to set to 640x480 Y8 at 30fps."
            "If your camera does not support this mode, please edit the source code and recompile the application."
            "Press Enter to exit.\n");
        return false;
    }



	///Buffers de imagen
	qDebug() << "BUFFERS DE IMAGEN ------------------------------------------";
	for ( int i=0; i < numCameras; i++ )
	{
		AimgBuffer[i] = ( Ipp8u * ) ippsMalloc_8u ( this->params.size * 9 );
		BimgBuffer[i] = ( Ipp8u * ) ippsMalloc_8u ( this->params.size * 9 );
		img8u_lum[i] = AimgBuffer[i];
		img8u_YUV[i] = AimgBuffer[i]+this->params.size;
		localYRGBImgBufferPtr[i] = BimgBuffer[i];
		qDebug() << "Reservando" << this->params.size * 9 <<" para localYRGBImgBufferPtr["<<i<<"]";
		printf("(de %p a %p)\n", localYRGBImgBufferPtr[i], localYRGBImgBufferPtr[i]+(this->params.size*9-1));
	}


	planos[0]=BimgBuffer[0]+this->params.size*3;
	planos[1]=BimgBuffer[0]+ ( this->params.size*4 );
	planos[2]=BimgBuffer[0]+ ( this->params.size*5 );
	img8u_aux = BimgBuffer[0]+(this->params.size*6);

	imgSize_ipp.width=this->params.width;
	imgSize_ipp.height=this->params.height;

	return true;
}

/// Interface methods

void Flycapture::getYUVPtr(uchar *buf, uchar cam, RoboCompCommonHead::THeadState &hState_ , RoboCompDifferentialRobot::TBaseState &bState_)
{
	mu->lock();
	memcpy ( buf, localYRGBImgBufferPtr[cam] + params.size, params.size*2 );
	hState_ = hState;
	bState_ = bState;
	mu->unlock();
}

void Flycapture::getYPtr(uchar *buf, uchar cam, RoboCompCommonHead::THeadState &hState_ , RoboCompDifferentialRobot::TBaseState &bState_)
{
	QMutexLocker locker ( mu );
	

	if (params.numCams > numCameras )
	{
		qFatal( "Flycapture::getYUVPtr() -> Fatal error. Not enough cameras in the bus");
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

void Flycapture::getYLogPolarPtr ( uchar *buf, uchar cam, RoboCompCommonHead::THeadState& hState,RoboCompDifferentialRobot::TBaseState& bState)
{
	RoboCompCommonHead::THeadState dummy;
	RoboCompDifferentialRobot::TBaseState dummy2;

	uchar buf1[params.height* params.height];

	getYPtr ( buf1, cam, dummy, dummy2 );
	lp->convert ( buf1,buf );

	hState = hState;
	bState = bState;
}

void Flycapture::getRGBPackedPtr ( uchar *buf, uchar cam, RoboCompCommonHead::THeadState& hState_ ,RoboCompDifferentialRobot::TBaseState& bState_)
{
	IppiSize imgSize_ipp = { params.width, params.height};
	
	qDebug()<<"begin getRGBPackedPtr";

	if (params.numCams > numCameras )
	{
		qFatal( "ieee1394capture::getYUVPtr() -> Fatal error. Not enough cameras in the bus");
		cleanup();
	}
	if ( ( cam == params.leftCamera )  or ( cam == params.rightCamera ))
	{
		mu->lock();
		ippiYUV422ToRGB_8u_C2C3R ( localYRGBImgBufferPtr[cam]+params.size, params.width*2, buf, params.width*3, imgSize_ipp);
		hState_ = hState;
		bState_ = bState;
		mu->unlock();
	}

	if (  cam == params.bothCameras )
	{
		mu->lock();
		ippiYUV422ToRGB_8u_C2C3R(localYRGBImgBufferPtr[params.leftCamera]+params.size,  params.width*2, buf,               params.width*3, imgSize_ipp );
		ippiYUV422ToRGB_8u_C2C3R(localYRGBImgBufferPtr[params.rightCamera]+params.size, params.width*2, buf+params.size*3, params.width*3, imgSize_ipp );
		hState_ = hState;
		bState_ = bState;
		mu->unlock();
	}


	
	qDebug()<<"end getRGBPackedPtr";
}

void Flycapture::getYRGBPtr (uchar *buf, uchar cam, RoboCompCommonHead::THeadState& hState_, RoboCompDifferentialRobot::TBaseState& bState_)
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


#endif

