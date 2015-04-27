/*
 *    Copyright (C) 2010 by RoboLab - University of Extremadura
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
#ifndef JOYSTICKHANDLER_H
#define JOYSTICKHANDLER_H

#include <math.h>
#include <QtCore>
#include <QIODevice>
#include <QTimer>

#include <Ice/Ice.h>

#include <OmniRobot.h>

#include <qjoystick/qjoystick.h>
#include <const.h>

using namespace std;

class JoyStickHandler : public QObject
{
Q_OBJECT
public:
	struct qjh_cfg_t
	{
		int32_t XMotionAxis;
		int32_t YMotionAxis;
		int32_t ZMotionAxis;
		int32_t maxAdvX, maxAdvZ;
		float maxRot;
		int32_t SampleRate;
	};

private:
	QMutex *mutex;
	struct qjh_joy_axis
	{
		float actualX;
		float actualY;
		float actualZ;
	};

	QTimer *jtimer; // Resend joy data to client
	qjh_joy_axis base_joy_axis;
	RoboCompOmniRobot::OmniRobotPrx base_proxy;

	QJoyStick *joystick;
	qjh_cfg_t config;

	bool buttonPressed;
	bool sendSpeed;
public:
	JoyStickHandler(qjh_cfg_t cfg, RoboCompOmniRobot::OmniRobotPrx base_proxy, QString joystick_device);
	~JoyStickHandler();

	bool open();

private slots:
	void receivedJoyStickEvent( int value, int type, int number );
	void sendJoyStickEvent();
};

#endif

