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

#include <genericworker.h>
#include <qjoystick/qjoystick.h>

#define CHECK_PERIOD 5000
#define JOYSTICK_PRECISION 0.05
#define MAX_AXIS 15
#define JOYSTICK_MAX_RAW 32767.
#define JOYSTICK_MIN_RAW -32767.
#define JOYtoROBOT_ROT	( 1 / JOYSTICK_MAX_RAW)
#define JOYtoROBOT_ADV	( 1 / JOYSTICK_MAX_RAW)
#define JOYSTICK_CENTER  0.1
#define DEFAULT_BASE_SERVER_PROXY_STRING       "DifferentialRobotProxy"
#define JOYSTICK_EVENT_TYPE_NULL     0x00
#define JOYSTICK_EVENT_TYPE_BUTTON   0x01
#define JOYSTICK_EVENT_TYPE_AXIS     0x02

/**
       \brief
       @author authorname
*/

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx, QObject *parent = 0);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

public slots:
 	void compute(); 	
	void receivedJoystickEvent(int value, int type, int number);
	void sendJoystickEvent();
	
private:

	struct axesParams{
		std::string name;
		int minRange;
		int maxRange;
		bool inverted;
	};
	
	struct joystickData{
		std::string device;
		int  numAxes;
		int numButtons;
		int basicPeriod;
		axesParams axes[MAX_AXIS]; 
	};
	
	QTimer *jtimer; // Resend joy data to client
	RoboCompJoystickAdapter::TData data;
	RoboCompJoystickAdapter::JoystickAdapterPrx joyAdapterPrx;
	QJoyStick *joystick;
	bool sendEvent;
	bool active;
	float normalize(float X, float A, float B, float C, float D);
	joystickData joystickParams;
};

#endif