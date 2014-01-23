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
#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

 ////muecashead////////
#define MAX_X 2700
#define MIN_X -4900

#define MAX_Y 2700
#define MIN_Y -4900

#define MAX_Z 2700
#define MIN_Z -4900

#define MAX_TILT 1200
#define MIN_TILT -1400



#include <rcdraw/rcdraw.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdlib.h>

#define STEPS_PER_REV 17960.
#include <genericworker.h>

/**
       \brief
       @author authorname
*/
using namespace cv;
class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	void setParams(RoboCompCommonBehavior::ParameterList params);
	QMutex *mutex;
	void resetHead();
	void stopHead();
	void setPanLeft(float pan);
	void setPanRight(float pan);
	void setTilt(float tilt);
	void setNeck(float neck);
	void saccadic2DLeft(float leftPan, float tilt);
	void saccadic2DRight(float rightPan, float tilt);
	void saccadic3D(float leftPan, float rightPan, float tilt);
	void saccadic4D(float leftPan, float rightPan, float tilt, float neck);
	void setNMotorsPosition(const RoboCompJointMotor::MotorGoalPositionList& listGoals);
	RoboCompCommonHead::THeadParams getHeadParams();
	void getHeadState(RoboCompCommonHead::THeadState& hState);
	bool isMovingHead();

		//MuecasHead
	RoboCompJointMotor::MotorParamsList getAllMotorParams();
	void  getAllMotorState(RoboCompJointMotor::MotorStateMap& mstateMap);
	void  setPosition(const RoboCompJointMotor::MotorGoalPosition& goal);
	
	//motor emotion
	void move    (float pos, const string& Name);
	void moveneck();
	void movemobileelements();

	//Joystick
	void sendData(const TData& data);
	float normalize(float X, float A, float B, float C, float D);

public slots:
 	void compute(); 	
	
	//button control emotion
	void moveangry(bool);
	void movehappy(bool);
	void movefear(bool);
	void movesad(bool);

	
private:
	
	QImage *qImageRGB, *qImageRGB2;
	RCDraw *Rgbcv, *Rgbcv2;
	int imgSize;
	IplImage* cvImage;
	IplImage* cvImage2;
	IplImage* cvrgb;
	IplImage* cvrgb2;
		
	QMutex *mutex_memory;
	bool headmoving;
	RoboCompCommonHead::THeadParams headParams;
	RoboCompJointMotor::MotorParamsList mplFaulhaber,mplMuecas, mplAll;
	RoboCompJointMotor::MotorList mlFaulhaber,mlMuecas,mlAll;
	RoboCompJointMotor::MotorStateMap stateFaulhaber,stateMuecas,stateAll;
	std::string neckMotorName;
	std::string tiltMotorName;
	std::string leftPanMotorName;
	std::string rightPanMotorName;
	float joy_x_antiguo;
	float joy_y_antiguo;
	float joy_z_antiguo;

	
	std::string mouthMotorName;
	std::string eyeBLeftUPMotorName;
	std::string eyeBLeftYAWMotorName;
	std::string eyeBRightUPMotorName;
	std::string eyeBRightYAWMotorName;
					
	RoboCompCamera::TCamParams paramsCamera;
	RoboCompCamera::imgType imgCam;  
    RoboCompDifferentialRobot::TBaseState bState;
    RoboCompCommonHead::THeadState hState;
	
	RoboCompIMU::DataImu datos;
	RoboCompIMU::Orientation angle;
	
};

#endif