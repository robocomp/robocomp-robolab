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
    /////////////////////////////////////////////
    ///             getRGBD
    ////////////////////////////////////////////
        // scape key = 27
        int cont = 3;
        do
        {
            if (!openDevice())
            {
                qDebug("Open Failed.");
                usleep(100);
                qDebug("Trying open Device %i times more",cont);
                cont --;
            }
            else
            {
                cont = 10;
            }
        }
        while (cont > 0 && cont != 10);
        
        if (cont > 0)
        {
            printf("Device Open");
            
            
            initializeStreams();

            ///INICIALIZACION ATRIBUTOS GENERALES
            registration=RoboCompRGBD::None; //A QUE INICIALIZO REGISTRATION????

            ///INICIALIZACION MUTEX
            usersMutex = new QMutex();
            RGBMutex = new QMutex();
            depthMutex = new QMutex();
            pointsMutex = new QMutex();
            ///INICIALIZACION ATRIBUTOS PARA PINTAR
            mColor = new uint16_t [IMAGE_WIDTH*IMAGE_HEIGHT*3];
            auxDepth = new uint8_t [IMAGE_WIDTH*IMAGE_HEIGHT*3];
            qImgDepth = new QImage(IMAGE_WIDTH,IMAGE_HEIGHT,QImage::Format_Indexed8);
            printf("\nStart moving around to get detected...\n(PSI pose may be required for skeleton calibration, depending on the configuration)\n");
            // 76 fps de fspf 33 fsp de openni
            setPeriod(15);
            
        /////////////////////////////////////////////
        ///             FSPF
        ////////////////////////////////////////////
            PlaneFilter::PlaneFilterParams filterParams;

            filterParams.maxPoints = 2000;
            filterParams.numSamples = 20000;
            filterParams.numLocalSamples = 80;
            filterParams.planeSize = 100;
            filterParams.WorldPlaneSize = 50;
            filterParams.minInlierFraction = 0.8;
            filterParams.maxError = 20;
            filterParams.numRetries = 2;
            filterParams.maxDepthDiff = 1800;
            // Parameters for polygonization
            filterParams.runPolygonization = false;
            filterParams.minConditionNumber = 0.1;
            //Thresholds for polygon merging
            //double maxCosineError;
            //float maxPolygonDist;
            //float maxOffsetDiff;
            //float minVisibilityFraction;
            filterParams.filterOutliers = true;
            planeFilter = new PlaneFilter( points, filterParams);
        }
        else
        {
            qFatal("I can't open device. RGBD is being used");
        }
}

bool SpecificWorker::openDevice()
{
	openniRc = OpenNI::initialize();
	if (openniRc != openni::STATUS_OK)
	{
		OpenNI::shutdown();
		qDebug("openNi2Comp: OpenNI initialize failed: \n%s\n", OpenNI::getExtendedError());
                return false;
	}

	openniRc = device.open(ANY_DEVICE);
	if (openniRc != openni::STATUS_OK)
	{
		OpenNI::shutdown();
		qDebug("openNi2Comp: Device open failed: \n%s\n", OpenNI::getExtendedError());
                return false;
	}

	if(device.isFile())
	{
		qDebug("Es un archivo");
	}
	else
	{
		qDebug("Es un dispositivo fisico");
	}
	
	return true;
}

void SpecificWorker::initializeStreams()
{
	if (!openStream(SENSOR_DEPTH, &depth)) qFatal("OPEN DEPTH STREAM FAILED!");
	if (!openStream(SENSOR_COLOR, &color)) qFatal("OPEN COLOR STREAM FAILED!");

	IMAGE_WIDTH=depth.getVideoMode().getResolutionX();
	IMAGE_HEIGHT=depth.getVideoMode().getResolutionY();
	
	//RESIZE POINTMAP
	pointsBuff.resize(IMAGE_WIDTH*IMAGE_HEIGHT);

	depthBuffer = new vector<float>(IMAGE_WIDTH*IMAGE_HEIGHT);
	colorBuffer = new vector<ColorRGB>(IMAGE_WIDTH*IMAGE_HEIGHT*3);
	depthImage = new vector<float>(IMAGE_WIDTH*IMAGE_HEIGHT);
	colorImage = new vector<Ice::Byte>(IMAGE_WIDTH*IMAGE_HEIGHT*3); //x3 para RGB888Pixel
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


/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
    	depth.stop(); depth.destroy();
	color.stop(); color.destroy();
	device.close();
	OpenNI::shutdown();

	delete depthBuffer;
	delete colorBuffer;

	delete depthImage;
	delete colorImage;

	delete qImgDepth;

	delete usersMutex;
	delete RGBMutex;
	delete depthMutex;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
}

void SpecificWorker::compute()
{
    static int co = 0;
    static QTime reloj = QTime::currentTime();
    readFrame();
    computeCoordinates();        

    static RoboCompDifferentialRobot::TBaseState bState;
    static RoboCompJointMotor::MotorStateMap hState;
    pointsBuff.copy(points);

    vector< vector3f > filteredPointCloud,pointsNormals , outlierCloud;
    vector< vector2i > pixelLocs;
    vector< PlanePolygon > polygons;

    planeFilter->GenerateFilteredPointCloud(points, filteredPointCloud, pixelLocs, pointsNormals, outlierCloud, polygons);
    
    RoboCompFSPF::OrientedPoints ops;
    for( int i =0; i<filteredPointCloud.size(); i++)
    {
            RoboCompFSPF::OrientedPoint op;
            op.x = filteredPointCloud[i].x;
            op.y = filteredPointCloud[i].y;
            op.z = filteredPointCloud[i].z;
            op.nx = pointsNormals[i].x;
            op.ny = pointsNormals[i].y;
            op.nz = pointsNormals[i].z;
            ops.push_back(op);		
    }
    fspf_proxy->newFilteredPoints(ops);

//    qDebug() << points.size() << filteredPointCloud.size() << outlierCloud.size();
    co++;
    if( reloj.elapsed() > 1000)
    {
	qDebug() << points.size() << filteredPointCloud.size() << outlierCloud.size();
        qDebug() << co << " fps";
        co = 0;
        reloj.restart();
    }
    pointsBuff.swap();       
}

void SpecificWorker::readFrame()
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
}

void SpecificWorker::readColor()
{
//	printf("readColor\n");
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

//	printf("%d %d\n", IMAGE_HEIGHT, IMAGE_WIDTH);
	RGBMutex->lock();
	memcpy(&colorImage->operator[](0),pixColor,IMAGE_HEIGHT*IMAGE_WIDTH*3);

	RGBMutex->unlock();
}

void SpecificWorker::computeCoordinates()
{
	//QMutexLocker l(pointsMutex); 57.00 43.00
	static const float fovW = 521;
	static const float fovH = 521;
	static const float flength_x = IMAGE_WIDTH / (2.f * tan( fovW / 2.0 ) );
	static const float flength_y = IMAGE_HEIGHT / (2.f * tan( fovH / 2.0 ) );
	
	for( int y=0 ; y< IMAGE_HEIGHT ; y++ ) 
	{
		for( int x=0 ; x<IMAGE_WIDTH ; x++ ) 
		{
			const int offset = y*IMAGE_WIDTH + x;
			const float z = float(pixDepth[offset]) / 1000.0;
		
			if( z < 0.1 ) 
			{
				//pixDepth[offset] = 8;
				pointsBuff[offset].x = NAN;
				pointsBuff[offset].y = NAN;
				pointsBuff[offset].z = NAN;
				pointsBuff[offset].w = NAN;
			} 
			else 
			{
				//(*depthImage)[offset] = z;
				pointsBuff[offset].x = (z * (x - IMAGE_WIDTH/2) / flength_x) * 1000.;
				pointsBuff[offset].y = (z * (y - IMAGE_HEIGHT/2) / flength_y) * 1000.;
				pointsBuff[offset].z = z * 1000.;
				pointsBuff[offset].w = 1.0;
			}
		}
		
	}
}

void SpecificWorker::normalizeDepth()
{
	for (int i=0; i<(IMAGE_HEIGHT*IMAGE_WIDTH); i++)
	{
		normalDepth[i]=255-(255*depthBuffer->operator[](i)/5/1000);
		if (normalDepth[i] > 255) normalDepth[i] = 255;
		if (normalDepth[i] < 0) normalDepth[i] = 0;
	}
}

//////////////////////////////
/// SERVANT
//////////////////////////////

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

Registration SpecificWorker::getRegistration ( ){
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
	//QMutexLocker l(pointsMutex);
	pointsBuff.copy(points);
}
