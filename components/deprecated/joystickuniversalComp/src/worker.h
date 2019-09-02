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

// #include <ipp.h>
#include <QtGui>
#include <stdint.h>
#include <qlog/qlog.h>

#include <JoystickAdapter.h>
#include <CommonBehavior.h>
#include <qjoystick/qjoystick.h>


#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100
#define JOYSTICK_PRECISION 0.05
#define MAX_AXIS 15

using namespace std;

/**
       \brief
       @author authorname
*/
class Worker : public QThread
{
Q_OBJECT
public:
	Worker(RoboCompJoystickAdapter::JoystickAdapterPrx joystickadapterprx, QObject *parent = 0);
	~Worker();
	void killYourSelf();
	void setPeriod(int p);
	void setParams(RoboCompCommonBehavior::ParameterList _params);
	
	QMutex *monitor_mutex; // Control access for monitor-related resources
	QMutex *w_mutex;                //Shared mutex with servant
	

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
	
	
	joystickData joystickParams;
	
	int Period;
	void run();

	QTimer *jtimer; // Resend joy data to client
	RoboCompJoystickAdapter::TData data;
	RoboCompJoystickAdapter::JoystickAdapterPrx joyAdapterPrx;
	
	QJoyStick *joystick;

	bool sendEvent;
	
	bool active;
	
	float normalize(float X, float A, float B, float C, float D);
	

public slots:
	void receivedJoystickEvent(int value, int type, int number);
	void sendJoystickEvent();
	
signals:
	void kill();
};

#endif
