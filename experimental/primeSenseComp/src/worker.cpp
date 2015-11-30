/*
 *    Copyright (C) 2006-2011 by RoboLab - University of Extremadura
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

// #include <ipp.h>
#include <QtGui>
#include <stdint.h>
#include <qlog/qlog.h>
#include <ni/XnCppWrapper.h>



// ------------------------------------------------------------------------------------------------------------------------
// Internal state
// ------------------------------------------------------------------------------------------------------------------------

struct Worker::Data
{
	int period;
	QTimer timer;
	RoboCompRGBD::Registration registration;
	
	xn::Context context;
	
	xn::DepthGenerator depthGen;
	XnMapOutputMode depthMode;
	RoboCompRGBD::DepthSeq depthMap1, depthMap2;
	RoboCompRGBD::DepthSeq * depthMapR, * depthMapW;
	
	xn::ImageGenerator colorGen;
	XnMapOutputMode colorMode;
	RoboCompRGBD::ColorSeq colorMap1, colorMap2;
	RoboCompRGBD::ColorSeq * colorMapR, * colorMapW;
	RoboCompJointMotor::MotorStateMap head;
	RoboCompDifferentialRobot::TBaseState base;
	RoboCompRGBD::PointSeq pointsMap;
	
	int doubleBuffer;
	
	void initializeNI();
};



void Worker::Data::initializeNI()
{
	// TODO: CHECK RETURN VALUES
	
	// Create the OpenNI context
	XnStatus status = XN_STATUS_OK;
	status = context.Init();
	
	if (status)
	{
		printf("\n");
		printf("xnGetStatusString: %s\n\n", xnGetStatusString(status));
		printf("xnGetStatusName: %s\n\n", xnGetStatusName(status));
		qFatal("Can't init OpenNi context\n");
	}
			
	
	// Create the depth generator
	status = depthGen.Create( context );
	if (status)
	{
		printf("\n");
		printf("xnGetStatusString: %s\n\n", xnGetStatusString(status));
		printf("xnGetStatusName: %s\n\n", xnGetStatusName(status));
		qFatal("Can't create depth generator\n");
	}
	depthMode.nXRes = XN_VGA_X_RES;
	depthMode.nYRes = XN_VGA_Y_RES;
	depthMode.nFPS = 30;


	status = depthGen.SetMapOutputMode( depthMode );
	if (status)
	{
		printf("\n");
		printf("xnGetStatusString: %s\n\n", xnGetStatusString(status));
		printf("xnGetStatusName: %s\n\n", xnGetStatusName(status));
		qFatal("Can't set Map Output mode\n");
	}


	depthMap1.resize( XN_VGA_X_RES * XN_VGA_Y_RES );
	depthMap2.resize( XN_VGA_X_RES * XN_VGA_Y_RES );
	depthMapW = &depthMap1;
	depthMapR = &depthMap2;
	
	// Create the color generator
	status = colorGen.Create( context );
	colorMode.nXRes = XN_VGA_X_RES;
	colorMode.nYRes = XN_VGA_Y_RES;
	colorMode.nFPS = 30;
	status = colorGen.SetMapOutputMode( colorMode );
	colorMap1.resize( XN_VGA_X_RES * XN_VGA_Y_RES );
	colorMap2.resize( XN_VGA_X_RES * XN_VGA_Y_RES );
	colorMapW = &colorMap1;
	colorMapR = &colorMap2;
	
	doubleBuffer=0;
	
	// Create the point cloud
	pointsMap.resize( XN_VGA_X_RES * XN_VGA_Y_RES);
	
	// Start generating
	depthGen.GetAlternativeViewPointCap().SetViewPoint( colorGen );
	status = context.StartGeneratingAll();
}



// -----------------------
// Internal methods
// -----------------------

/**
* \brief Default constructor
*/
Worker::Worker(int fps, DifferentialRobotPrx differential_, JointMotorPrx joint_)
{
	mutex = new QMutex();
	joint = joint_;
	differential = differential_;
	// Start the compute thread
	d = new Worker::Data();
	d->initializeNI();
	d->period = 1000./fps;
	connect( &d->timer, SIGNAL(timeout()), this, SLOT(compute()) );
	d->timer.start( d->period );

	puntos=true;
	///luiky
	if(puntos==false)
	{
		step_depth=0;
		depthLuiky= ippiMalloc_32f_C1(640,480,&step_depth);
		ippSetNumThreads( 1) ;
	}
	

}

/**
* \brief Default destructor
*/
Worker::~Worker()
{
	delete d;
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
	d->period = p;
	d->timer.start( d->period );
}



/**
* \brief
* @param params_ Parameter list received from monitor thread
*/
bool Worker::setParams(RoboCompCommonBehavior::ParameterList params_)
{
//	active = false;
		//CAMBIAR PARAMETROS Y RE-ARRANQUE DEL COMPONENTE SI ES NECESARIO

//	active = true;
	return true;
}



void Worker::compute()
{
	//qDebug()<<"compute";
	d->context.WaitAndUpdateAll();
	const XnDepthPixel* depthTmp = d->depthGen.GetDepthMap();
	const XnRGB24Pixel* colorTmp = d->colorGen.GetRGB24ImageMap();

// 	try
// 	{
// 		differential->getBaseState(d->base);
// 		joint->getAllMotorState(d->head);
// 	}
// 	catch(...)
// 	{
// 	  qDebug() << "PrimeSenseComp: error: can't read base odometry and/or joint motor positions";
// 	}
	
	if (puntos )
	{
	XnFieldOfView fov;
	d->depthGen.GetFieldOfView( fov );
	const float flength_x = XN_VGA_X_RES / (2 * tan( fov.fHFOV / 2.0 ) );
	const float flength_y = XN_VGA_Y_RES / (2 * tan( fov.fVFOV / 2.0 ) );
	
	for( int y=0 ; y<XN_VGA_Y_RES ; y++ ) {
		for( int x=0 ; x<XN_VGA_X_RES ; x++ ) {
			const int offset = y*XN_VGA_X_RES + x;
			const float z = float(depthTmp[offset]) / 1000.0;
			if( z < 0.1 ) {
				(*d->depthMapW)[offset] = NAN;
				d->pointsMap[offset].x = NAN;
				d->pointsMap[offset].y = NAN;
				d->pointsMap[offset].z = NAN;
				d->pointsMap[offset].w = NAN;
			} else {
				(*d->depthMapW)[offset] = z;
				d->pointsMap[offset].x =  (z * (x - XN_VGA_X_RES/2) / flength_x) * 1000.;
				d->pointsMap[offset].y = -(z * (y - XN_VGA_Y_RES/2) / flength_y) * 1000.;
				d->pointsMap[offset].z = z * 1000.;
				d->pointsMap[offset].w = 1.0;
			}
		}
	}
	
	}
	else
	{
		IppiSize ippSizeImage;
		ippSizeImage.width=640;
		ippSizeImage.height=480;
		//en los float caben los unsigned
		//qDebug()<<"explota";
		
		ippiConvert_16u32f_C1R (depthTmp,640*sizeof(Ipp16u),depthLuiky,step_depth,ippSizeImage);
		//qDebug()<<"explota";
		//a metros;
		ippiDivC_32f_C1IR((Ipp32f)1000,depthLuiky,step_depth,ippSizeImage);
		memcpy( &((*d->depthMapW)[0]), depthLuiky, XN_VGA_X_RES*XN_VGA_Y_RES*sizeof(float) );


	}
 	//memcpy( &((*d->depthMapW)[0]), depthTmp, XN_VGA_X_RES*XN_VGA_Y_RES*sizeof(XnDepthPixel) );
        
	memcpy( &((*d->colorMapW)[0]), colorTmp, XN_VGA_X_RES*XN_VGA_Y_RES*sizeof(XnRGB24Pixel) );
	
	mutex->lock();
		if(d->doubleBuffer==0)
		{
			d->colorMapW = &d->colorMap2;
			d->depthMapW = &d->depthMap2;
			d->colorMapR = &d->colorMap1;
			d->depthMapR = &d->depthMap1;
			d->doubleBuffer = 1;
		}
		else
		{
  			d->colorMapW = &d->colorMap1;
			d->depthMapW = &d->depthMap1;
			d->colorMapR = &d->colorMap2;
			d->depthMapR = &d->depthMap2;
			d->doubleBuffer = 0;

		}
	mutex->unlock();
	//qDebug()<<"*-----";
	//printf( "Middle pixel depth: %f\n", d->depthMap[153600] );
	printFPS();
}



void Worker::printFPS()
{
	static int fps=0;
	static QTime ti(0,0,0);
	static QTime tt=QTime::currentTime();
	static QTime foo(0,0,0);
	if ((fps++ % 50) == 0) {
		uint32_t e=ti.restart();
		if (e) {
			int v = 50000 / e;
			qDebug() << "PrimeSenseComp Fps: " << v << "Elapsed:" << foo.addMSecs( tt.elapsed()).toString("hh:mm:ss:zzz");
		}
	}
}



// ------------------------------------------------------------------------------------------------------------------------
// RGBD interface
// ------------------------------------------------------------------------------------------------------------------------

RoboCompRGBD::TRGBDParams Worker::getRGBDParams()
{
	// TODO: implementar
}



void Worker::setRegistration(RoboCompRGBD::Registration registration)
{
	mutex->lock();
	d->registration = registration;
	switch( registration ) {
		case RoboCompRGBD::None:
			d->colorGen.GetAlternativeViewPointCap().ResetViewPoint();
			d->depthGen.GetAlternativeViewPointCap().ResetViewPoint();
			break;
		case RoboCompRGBD::DepthInColor:
			d->colorGen.GetAlternativeViewPointCap().ResetViewPoint();
			d->depthGen.GetAlternativeViewPointCap().SetViewPoint( d->colorGen );
			break;
		case RoboCompRGBD::ColorInDepth:
			d->colorGen.GetAlternativeViewPointCap().SetViewPoint( d->depthGen );
			d->depthGen.GetAlternativeViewPointCap().ResetViewPoint();
			break;
	}
	mutex->unlock();
}



RoboCompRGBD::Registration Worker::getRegistration()
{
	return d->registration;
}


void Worker::getData(RoboCompRGBD::imgType& rgbMatrix, RoboCompRGBD::depthType& distanceMatrix, MotorStateMap& hState, TBaseState& bState)
{
	mutex->lock();
	//rgbMatrix= (RoboCompRGBD::imgType *) (d->colorMapR[0]);
	rgbMatrix.resize(XN_VGA_X_RES*XN_VGA_Y_RES*3);
	memcpy( &rgbMatrix[0],&((*d->colorMapR)[0]), XN_VGA_X_RES*XN_VGA_Y_RES*3 );
	distanceMatrix= *d->depthMapR;
	hState = d->head;
	bState = d->base;
	
	mutex->unlock();
	
}

void Worker::getImage(RoboCompRGBD::ColorSeq& color, RoboCompRGBD::DepthSeq& depth, RoboCompRGBD::PointSeq& points, RoboCompJointMotor::MotorStateMap &head, RoboCompDifferentialRobot::TBaseState& base )
{
	mutex->lock();
	color = *d->colorMapR;
	depth = *d->depthMapR;	
	points = d->pointsMap;
	head = d->head;
	base = d->base;
	mutex->unlock();
}



void Worker::getDepth(RoboCompRGBD::DepthSeq& depth, RoboCompJointMotor::MotorStateMap &head, RoboCompDifferentialRobot::TBaseState& base )
{
	mutex->lock();
	depth = *d->depthMapR;
	mutex->unlock();
}



void Worker::getRGB(RoboCompRGBD::ColorSeq& color, RoboCompJointMotor::MotorStateMap &head, RoboCompDifferentialRobot::TBaseState& base )
{
	mutex->lock();
	color = *d->colorMapR;
	mutex->unlock();
}



void Worker::getXYZ(RoboCompRGBD::PointSeq& points, RoboCompJointMotor::MotorStateMap &head, RoboCompDifferentialRobot::TBaseState& base )
{
	mutex->lock();
	points = d->pointsMap;
	mutex->unlock();
}

