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
#ifndef WORKER_H
#define WORKER_H

#include <ipp.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <PvApi.h>
#include <QtGui>
#include <stdint.h>
#include <stdlib.h>
#include <qlog/qlog.h>
#include <CommonBehavior.h>
#include <CameraBus.h>
#include <DifferentialRobot.h>
#include <JointMotor.h>

#define FRAMESCOUNT 15
#define BASIC_PERIOD 100
#define MAX_CAMERAS 5
#define MAX_WIDTH 800
#define MAX_HEIGHT 600
 

using namespace RoboCompCameraBus;
using namespace std;

/**
       \brief
       @author authorname
*/
class Worker  : public QThread
{
Q_OBJECT
public: 
	Worker(RoboCompDifferentialRobot::DifferentialRobotPrx differentialrobotprx, RoboCompJointMotor::JointMotorPrx jointmotorprx);
	~Worker();
	bool init( );
	//CommonBehavior
	void killYourSelf();
	void setPeriod(int p);
	bool setParams(RoboCompCommonBehavior::ParameterList params_);
	//CameraBus
	void getImage(const string& camera, const RoboCompCameraBus::Format& format, RoboCompCameraBus::Image& image);
	void getSyncImages(const RoboCompCameraBus::CameraList& cameraList, const RoboCompCameraBus::Format& format, bool all,RoboCompCameraBus::ImageList& imagelist);
	void getBaseState (RoboCompDifferentialRobot::TBaseState &bState);
	void getHeadState (RoboCompJointMotor::MotorStateMap &hState);

	
private:
	void run();  //Thread
	//Prosilica
	void Sleep(unsigned int time);
	// wait for a camera to be plugged
	int WaitForCamera();
	// wait forever (at least until there is no more camera
	void WaitForEver();
	// get the first camera found
	bool CameraGrab();
	// open the camera
	bool CameraSetup();
	// setup and start streaming
	bool CameraStart() ;
	// stop streaming
	void CameraStop();
	// unsetup the camera
	void CameraUnsetup() ;

	bool grab();
	
	
	/**
	 *    Utility class for printing current frame rate.
	 */
	inline void printFPS( )
	{
		static int fps=0;
		static QTime ti(0,0,0);
		static QTime tt=QTime::currentTime();
		static QTime foo(0,0,0);
		if ((++fps % 50) == 0)
		{
			uint32_t e=ti.restart();
			if (e)
			{
				int v = 50000 / e;
				qDebug() << "CamaraComp Fps: " << v << "Elapsed:" << foo.addMSecs( tt.elapsed()).toString("hh:mm:ss:zzz");
			}
		}
	}
	
	
	
	
public:
	RoboCompCameraBus::BusParams busparams;		 ///*<params structure holding camera bus params
	RoboCompCameraBus::CameraParamsList cameraParamsList;
	RoboCompCameraBus::CameraList cameraList;
	RoboCompCameraBus::Format cameraFormat;

private:
	int Period;
	QMutex *mu;                                    ///*< mu mutex for double buffer management
	bool _finished;
	
	RoboCompCameraBus::FormatNotAvailable ex;
	RoboCompDifferentialRobot::DifferentialRobotPrx base;                    ///*<base proxy to BaseComp
	RoboCompDifferentialRobot::TBaseState  bState,bStateAfter, bStateBefore; ///*<bState structure maintained by BaseComp holding base instantaneous kinematics
	RoboCompJointMotor::JointMotorPrx head;                                  ///*<head  proxy to JointMotorComp
	RoboCompJointMotor::MotorStateMap map;
	RoboCompJointMotor::MotorList motorList;	                             ///*<params structure maintained by JointMotor to hold a list of motor names
	RoboCompJointMotor::MotorStateMap hState,hStateAfter, hStateBefore;      ///*<hState structure maintained by JointMotor holding head instantaneous kinematics
	RoboCompJointMotor::MotorStateMap motorStateMap;                         ///*<params structure maintained by JointMotor to return a list of motor states
	
	//Image memory allocation
	uchar *imgBufferGrabA[MAX_CAMERAS];
	uchar *imgBufferGrabB[MAX_CAMERAS];
	uchar *imgBufferGrabPtr[MAX_CAMERAS];
	uchar *imgBufferSour[MAX_CAMERAS];
	uchar *imgBufferTransformA[MAX_CAMERAS];
	uchar *imgBufferTransformB[MAX_CAMERAS];
	uchar *pSour[MAX_CAMERAS];
	uchar *pDes[MAX_CAMERAS];
	uchar *buf;
	
	uchar *img8u_lum[MAX_CAMERAS];
	uchar *img8u_YUV[MAX_CAMERAS];
	
	
	//Prosilica
	struct tCamera
	{
		unsigned long   UID;
		tPvHandle       Handle;
		tPvFrame	    Frame;
		bool            Abort;
	};

	// global camera data
	tCamera *GCamera;
	
signals:
	void kill();
};

#endif
