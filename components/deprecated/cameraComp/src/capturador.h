/*
 *    Copyright (C) 2008-2010 by RoboLab - University of Extremadura
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
#ifndef CAPTURADOR_H
#define CAPTURADOR_H

#include <stdint.h>
#include <QtCore>

#ifdef USE_IPP
#include <ipp.h>
#else
#include <ippWrapper.h>
#endif

#include <iostream>
#include <logpolar/lpolar.h>
#include <Camera.h>
#include <CommonHead.h>
#include <DifferentialRobot.h>

#define MAX_CAMERAS 5

/**
	\class Capturador <p>Abstract class of CamaraComp component. It defines a generic interface for a cameras server. Real camera handlers should be implemented as a derived class</p>
 */

class Capturador : public QThread
{
Q_OBJECT
public:
	Capturador()
	{
		mu = new QMutex(QMutex::Recursive);
	}
	~Capturador()
	{
	}

	/**
	 * Virtual method for objects initialization
	 * @param params structure with camera parameters
	 * @param headPrx proxy to JointMotorComp
	 * @param basePrx proxy to BaseComp
	 * @return true if initialization succeed
	 */
	virtual bool init(RoboCompCamera::TCamParams&, RoboCompJointMotor::JointMotorPrx, RoboCompDifferentialRobot::DifferentialRobotPrx)=0;
	virtual void run()=0;

	//Metodos que dan servicio al interfaz del componente
	virtual void getYUVPtr(uchar*, uchar, RoboCompCommonHead::THeadState&, RoboCompDifferentialRobot::TBaseState&)=0;
	virtual void getYRGBPtr(uchar*, uchar, RoboCompCommonHead::THeadState&, RoboCompDifferentialRobot::TBaseState&)=0;
	virtual void getYPtr(uchar*, uchar, RoboCompCommonHead::THeadState&, RoboCompDifferentialRobot::TBaseState &)=0;
	virtual void getRGBPackedPtr(uchar*, uchar, RoboCompCommonHead::THeadState&, RoboCompDifferentialRobot::TBaseState&)=0;
	virtual void getYLogPolarPtr(uchar*, uchar, RoboCompCommonHead::THeadState&, RoboCompDifferentialRobot::TBaseState&)=0;

	//Buffers para double buffering. Quizas haya que quitarlo de aqui si sólo se usa en un driver
	Ipp8u *AimgBuffer[MAX_CAMERAS];
	Ipp8u *BimgBuffer[MAX_CAMERAS];
	Ipp8u *img8u_lum[MAX_CAMERAS];
	Ipp8u *img8u_YUV[MAX_CAMERAS];
	Ipp8u *planos[3];
	Ipp8u *img8u_aux;
	IppiSize imgSize_ipp;
	Ipp8u* localYRGBImgBufferPtr[MAX_CAMERAS];

	RoboCompJointMotor::JointMotorPrx head;        ///*<head  proxy to JointMotorComp
	RoboCompDifferentialRobot::DifferentialRobotPrx base;                  ///*<base proxy to BaseComp
	RoboCompCommonHead::THeadState hState, hStateAfter, hStateBefore;        ///*<hState structure maintained by JointMotor holding head instantaneous kinematics
	RoboCompDifferentialRobot::TBaseState bState, bStateAfter, bStateBefore;             ///*<bState structure maintained by BaseComp holding base instantaneous kinematics
	RoboCompCamera::TCamParams params;           ///*<params structure maintained by CamaraComp holding camera params
	RoboCompJointMotor::MotorList motorList;	 ///*<params structure maintained by JointMotor to hold a list of motor names
	RoboCompCommonHead::THeadState motorStateMap;  ///*<params structure maintained by JointMotor to return a list of motor states

	QMutex *mu;                                    ///*< mu mutex for double buffer management
	bool _finished;                                ///*< _finished for sending the thread to sleep when no work
	bool sleeped;
	QTime reloj;                                   ///*< Reloj for timeStamps
	std::string timeStamp;                         ///*< timeStamp for timeStamping the images

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

	LPolar *lp;                                    ///*< lp pointer to log-polar class
protected:
	 int numCameras;
};

#endif
