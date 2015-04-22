/*
 *    Copyright (C) 2015 by YOUR NAME HERE
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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	openDevice();
	initializeStreams();
//	initializeTracking();

	///INICIALIZACION MUTEX
	usersMutex = new QMutex();
	RGBMutex = new QMutex();
	depthMutex = new QMutex();

	setPeriod(33);
}



/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	depth.stop(); depth.destroy();
	color.stop(); color.destroy();
	device.close();
	OpenNI::shutdown();

	delete usersMutex;
	delete RGBMutex;
	delete depthMutex;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//       THE FOLLOWING IS JUST AN EXAMPLE
//
// 	try
// 	{
// 		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
// 		innermodel_path=par.value;
// 		innermodel = new InnerModel(innermodel_path);
// 	}
// 	catch(std::exception e) { qFatal("Error reading config params"); }
	
	
	timer.start(Period);

	return true;
}

void SpecificWorker::compute()
{
	openni::Status rc = openni::STATUS_OK;

	openni::VideoStream* streams[] = {&depth, &color};

	int changedIndex = -1;
	while (rc == openni::STATUS_OK)
	{
		rc = openni::OpenNI::waitForAnyStream(streams, 2, &changedIndex, 0);
		if (rc == openni::STATUS_OK)
		{
			switch (changedIndex)
			{
			case 0:
				readDepth(); break;
			case 1:
				readColor(); break;
			default:
				printf("Error in wait\n");
			}
		}
	}
}

void SpecificWorker::readColor()
{
	printf("readColor\n");
	openniRc = color.readFrame(&colorFrame);
	if (openniRc != openni::STATUS_OK)
	{
		printf("Read color failed!\n%s\n", OpenNI::getExtendedError());
		return;
	}

	if (colorFrame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_RGB888)
	{
		printf("Unexpected color frame format\n");
		return;
	}

	uint8_t *pixColor = (uint8_t *)colorFrame.getData();

	printf("%d %d\n", IMAGE_HEIGHT, IMAGE_WIDTH);
	RGBMutex->lock();
	
//		memcpy(&colorImage->operator[](0),pixColor,IMAGE_HEIGHT*IMAGE_WIDTH*3);
		memcpy(&colorImageB[0],pixColor,IMAGE_HEIGHT*IMAGE_WIDTH*3);

	
// 	uint64_t sss = 0;
// 	for (int i=0; i<IMAGE_HEIGHT*IMAGE_WIDTH*3; i++)
// 	{
// 		sss += (uint8_t)pixColor[i];
// 	}
// 	std::cout << "sum " << sss << "\n";
	RGBMutex->unlock();
}


void SpecificWorker::readDepth()
{
	openniRc = depth.readFrame(&depthFrame);
	if (openniRc != openni::STATUS_OK)
	{
		printf("Read depth failed!\n%s\n", OpenNI::getExtendedError());
		return;
	}

	if (depthFrame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_1_MM && depthFrame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_100_UM)
	{
		printf("Unexpected depth frame format\n");
		return;
	}

	pixDepth = (DepthPixel*)depthFrame.getData();
	///¿PUEDE HACERSE CON MEMCPY? DISTINTOS TIPOS... (short, float). Creo que NO
	//memcpy(&depthBuffer->operator[](0),pixDepth,IMAGE_WIDTH*IMAGE_HEIGHT*sizeof(float));	//ESTO SUSTITUIRÍA AL FOR DE DEBAJO
	
 	for(int i=0;i<IMAGE_WIDTH*IMAGE_HEIGHT;i++)
	//	depthBuffer->operator[](i)=pixDepth[i];
		depthBufferB[i]=pixDepth[i];
// 	
// 	
 	depthMutex->lock();
	//	depthImage=depthBuffer;
		//memcpy(&depthImage->operator[](0),&depthBuffer->operator[](0),IMAGE_WIDTH*IMAGE_HEIGHT*sizeof(float)); //ESTO SUSTITUIRÍA AL = DE ARRIBA
	depthMutex->unlock();
}

void SpecificWorker::openDevice()
{
	openniRc = OpenNI::initialize();
	if (openniRc != openni::STATUS_OK)
	{
		OpenNI::shutdown();
		qFatal("openNi2Comp: OpenNI initialize failed: \n%s\n", OpenNI::getExtendedError());
	}

	openniRc = device.open(ANY_DEVICE);
		//   openniRc = device.open("/home/robocomp/robocomp/components/programaposturas/Prueba.oni");
	if (openniRc != openni::STATUS_OK)
	{
		OpenNI::shutdown();
		qFatal("openNi2Comp: Device open failed: \n%s\n", OpenNI::getExtendedError());
	}

	if(device.isFile())
	{
		qDebug("Es un archivo");
	}
	else
	{
		qDebug("Es un dispositivo fisico");
	}
}

void SpecificWorker::initializeStreams()
{
	if (!openStream(SENSOR_DEPTH, &depth)) qFatal("OPEN DEPTH STREAM FAILED!");
	if (!openStream(SENSOR_COLOR, &color)) qFatal("OPEN COLOR STREAM FAILED!");

	IMAGE_WIDTH=depth.getVideoMode().getResolutionX();
	IMAGE_HEIGHT=depth.getVideoMode().getResolutionY();

	
// 	depthBuffer = new vector<float>(IMAGE_WIDTH*IMAGE_HEIGHT);
// 	colorBuffer = new vector<ColorRGB>(IMAGE_WIDTH*IMAGE_HEIGHT*3);
// 	depthImage = new vector<float>(IMAGE_WIDTH*IMAGE_HEIGHT);
// 	colorImage = new vector<Ice::Byte>(IMAGE_WIDTH*IMAGE_HEIGHT*3); //x3 para RGB888Pixel
// 	
 	depthBufferB.resize(IMAGE_WIDTH*IMAGE_HEIGHT);
 	depthImageB.resize(IMAGE_WIDTH*IMAGE_HEIGHT);
 	colorImageB.resize(IMAGE_WIDTH*IMAGE_HEIGHT*3); //x3 para RGB888Pixel
}

bool SpecificWorker::openStream(SensorType sensorType, VideoStream *stream)
{
	openniRc = stream->create(device, sensorType);
	if (openniRc != openni::STATUS_OK)
	{
		printf("openNi2Comp: Couldn't find stream:\n%s\n", OpenNI::getExtendedError());
		return false;
	}
	openniRc = stream->start();
	if (openniRc != openni::STATUS_OK)
	{
		printf("openNi2Comp: Couldn't start stream:\n%s\n", OpenNI::getExtendedError());
		stream->destroy();
		return false;
	}
	if (!stream->isValid())
		return false;

	return true;
}


///////////////////////////////////////////////////////
///RGBDBus
///////////////////////////////////////////////////////

RoboCompRGBDBus::CameraParamsMap SpecificWorker::getAllCameraParams()
{
}

void SpecificWorker::getPointClouds(const CameraList &cameras, PointCloudMap &clouds)
{
}

void SpecificWorker::getImages(const CameraList &cameras, ImageMap &images)
{
	images[cameras[0]].camera;
	
	depthMutex->lock();
		images[cameras[0]].depthImage = depthBufferB;
	depthMutex->unlock();
	
	RGBMutex->lock();
		images[cameras[0]].colorImage = colorImageB;
	RGBMutex->unlock();
	images[cameras[0]].width = IMAGE_WIDTH;
	images[cameras[0]].height = IMAGE_HEIGHT;
}

void SpecificWorker::getProtoClouds(const CameraList &cameras, PointCloudMap &protoClouds)
{
}

void SpecificWorker::getDecimatedImages(const CameraList &cameras, const int decimation, ImageMap &images)
{
}

///////////////////////////////////////////////////////
///RGBD
///////////////////////////////////////////////////////

TRGBDParams SpecificWorker::getRGBDParams( )
{
	//¿CAMBIA ALGO DE ESTO DE UN FRAME A OTRO?
	TRGBDParams params;
	//params.color.focal=;
	params.color.width=IMAGE_WIDTH;
	params.color.height=IMAGE_HEIGHT;
	params.color.size=IMAGE_WIDTH*IMAGE_HEIGHT*3;
	//params.color.FPS=;
	//params.depth.focal=;
	params.depth.width=IMAGE_WIDTH;
	params.depth.height=IMAGE_HEIGHT;
	params.depth.size=IMAGE_WIDTH*IMAGE_HEIGHT;
	//params.depth.FPS=;¡
	params.timerPeriod=Period;
	//params.talkToBase=;
	//params.talkToJointMotor=;
	//params.driver=;
	//params.device=;
	return params;
}

void SpecificWorker::setRegistration (RoboCompRGBD::Registration value)
{
	registration=value;
}

Registration SpecificWorker::getRegistration ( )
{
	return registration;
}

void SpecificWorker::getData(imgType& rgbMatrix, depthType& distanceMatrix, RoboCompJointMotor::MotorStateMap& hState, RoboCompDifferentialRobot::TBaseState& bState)
{
	RGBMutex->lock();
		rgbMatrix=*colorImage;
	RGBMutex->unlock();
	depthMutex->lock();
		distanceMatrix=*depthImage;
	depthMutex->unlock();

}

void SpecificWorker::getDepthInIR(depthType& distanceMatrix, RoboCompJointMotor::MotorStateMap &hState, RoboCompDifferentialRobot::TBaseState& bState)
{
	qDebug()<<"getDepthInIR not implemented yet";
}

void SpecificWorker::getImage(ColorSeq& color, DepthSeq& depth, PointSeq& points, RoboCompJointMotor::MotorStateMap &hState, RoboCompDifferentialRobot::TBaseState& bState)
{
	getDepth(depth,hState,bState);
	getRGB(color,hState,bState);
	getXYZ(points,hState,bState);
}

void SpecificWorker::getDepth(DepthSeq& depth, RoboCompJointMotor::MotorStateMap &hState, RoboCompDifferentialRobot::TBaseState& bState )
{
	depthMutex->lock();
		depth=*depthBuffer;
	depthMutex->unlock();
}

void SpecificWorker::getRGB(ColorSeq& color, RoboCompJointMotor::MotorStateMap &hState, RoboCompDifferentialRobot::TBaseState& bState)
{
	RGBMutex->lock();
		color=*colorBuffer;
	RGBMutex->unlock();
}

void SpecificWorker::getXYZ(PointSeq& points, RoboCompJointMotor::MotorStateMap &hState, RoboCompDifferentialRobot::TBaseState& bState)
{
	qDebug()<<"getXYZ not implemented yet";
}



