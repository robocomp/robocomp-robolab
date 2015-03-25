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
#include "joystickhandler.h"

JoyStickHandler::JoyStickHandler(qjh_cfg_t cfg,RoboCompOmniRobot::OmniRobotPrx _base_proxy, QString joystick_device )
{
	config = cfg;
	base_proxy = _base_proxy;
	try
	{
//		RoboCompOmniRobot::TMechParams mparams;
//		mparams = base_proxy->getParams();
// 		if(mparams.maxVelAdv < config.maxAdv)
// 			config.maxAdv = mparams.maxVelAdv;
// 		if(mparams.maxVelRot < config.maxRot)
// 			config.maxRot = mparams.maxVelRot;
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Failure reading differentialRobot params. Using maxAdv and maxSteer values from config" << endl;
	}
	// Set the base joystick axis initial data
	base_joy_axis.actualX = 0.;
	base_joy_axis.actualY = 0.;
	base_joy_axis.actualZ = 0.;

	joystick = new QJoyStick(joystick_device, 3);
	jtimer = new QTimer();
	sendSpeed = false;

	// Connect signals
	connect( joystick, SIGNAL( inputEvent(int, int, int) ), this, SLOT( receivedJoyStickEvent(int, int, int) ) );
	connect( jtimer, SIGNAL( timeout() ), this, SLOT( sendJoyStickEvent() ) );
	qWarning("[%s]: New JoyStick Handler settings: XMotionAxis [%2d], YMotionAxis [%2d], ZMotionAxis [%2d]", PROGRAM_NAME, config.XMotionAxis, config.YMotionAxis, config.ZMotionAxis);
	qWarning("[%s]: Max advance speed: [%i, %i], Max steering speed: [%f]",PROGRAM_NAME, config.maxAdvX, config.maxAdvZ, config.maxRot);
}

JoyStickHandler::~JoyStickHandler()
{
	jtimer->stop();
	joystick->stop();
	
	disconnect ( joystick );
	disconnect ( jtimer );
	
	delete jtimer;
	delete joystick;
}

bool JoyStickHandler::open()
{
	if (joystick->openQJoy())
	{
		joystick->start();
		if (config.SampleRate < 1) config.SampleRate = 1;
		jtimer->start( 1000 / config.SampleRate );
		return TRUE;
	}
	else
		return FALSE;
}

void JoyStickHandler::receivedJoyStickEvent(int value, int type, int number)
{
	if (type == JOYSTICK_EVENT_TYPE_AXIS)
	{
		if (number == config.XMotionAxis)
		{	
			if (fabs(value) > JOYSTICK_CENTER)
				base_joy_axis.actualX = JOYtoROBOT_ROT * value;
			else
				base_joy_axis.actualX = 0.f;
// 			cout << "[" << PROGRAM_NAME << "]: Motion in axis X: "<< base_joy_axis.actualX<<endl;
		}
		else if (number == config.YMotionAxis)
		{
			if (fabs(value) > JOYSTICK_CENTER)
				base_joy_axis.actualY = JOYtoROBOT_ROT * -value;
			else
				base_joy_axis.actualY = 0.f;
// 			cout << "[" << PROGRAM_NAME << "]: Motion in axis Y: "<< base_joy_axis.actualY<<endl;
		}
		else if (number == config.ZMotionAxis)
		{
			if (fabs(value) > JOYSTICK_CENTER)
				base_joy_axis.actualZ = JOYtoROBOT_ROT * value;
			else
				base_joy_axis.actualZ = 0.f;
// 			cout << "[" << PROGRAM_NAME << "]: Motion in axis Z: "<< base_joy_axis.actualZ<<endl;
		}
		sendSpeed = true;
	}
}

void JoyStickHandler::sendJoyStickEvent()
{
	const float xv = base_joy_axis.actualX*config.maxAdvX;
	const float zv = base_joy_axis.actualY*config.maxAdvZ;
	const float rv = base_joy_axis.actualZ*config.maxRot;
	printf("%d (%f %f) (%f)\n", sendSpeed, xv, zv, rv);

	try
	{
		if (sendSpeed)
		{
			if (fabs(xv)>10 or fabs(zv)>10 or fabs(rv)>0.02)
				base_proxy->setSpeedBase(xv, zv, rv);
		}
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Fallo la comunicacion a traves del proxy (base). Waiting" << endl;
		cout << "[" << PROGRAM_NAME << "]: Motivo: " << endl << ex << endl;
	}
}

