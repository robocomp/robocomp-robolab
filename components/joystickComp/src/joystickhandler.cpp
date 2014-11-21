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

JoyStickHandler::JoyStickHandler(qjh_cfg_t cfg,RoboCompDifferentialRobot::DifferentialRobotPrx base_proxy, QString joystick_device )
{
	config = cfg;
	_base_proxy = base_proxy;
	try{
//		RoboCompDifferentialRobot::TMechParams mparams;
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

	joystick = new QJoyStick( joystick_device );
	jtimer = new QTimer( );
	sendSpeed = false;

	// Connect signals
	connect( joystick, SIGNAL( inputEvent(int, int, int) ), this, SLOT( receivedJoyStickEvent(int, int, int) ) );
	connect( jtimer, SIGNAL( timeout() ), this, SLOT( sendJoyStickEvent() ) );
	qWarning("[%s]: New JoyStick Handler settings: XMotionAxis [%2d], YMotionAxis [%2d]", PROGRAM_NAME, config.XMotionAxis, config.YMotionAxis);
	qWarning("[%s]: Max advance speed: [%i], Max steering speed: [%f]",PROGRAM_NAME,config.maxAdv,config.maxRot);
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
	if ( joystick->openQJoy() )
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
	if ( type == JOYSTICK_EVENT_TYPE_AXIS )
	{
		if ( number == config.XMotionAxis )
		{
			if( fabs(value) > JOYSTICK_CENTER )
				base_joy_axis.actualX = JOYtoROBOT_ROT * value;
			else
				base_joy_axis.actualX = 0.f;
			cout << "[" << PROGRAM_NAME << "]: Motion in axis X: "<< base_joy_axis.actualX<<endl;
		}
		else if ( number == config.YMotionAxis )
		{
			if( fabs(value) > JOYSTICK_CENTER )
				base_joy_axis.actualY = JOYtoROBOT_ROT * -value;
			else
				base_joy_axis.actualY = 0.f;
			cout << "[" << PROGRAM_NAME << "]: Motion in axis Y: "<< base_joy_axis.actualY<<endl;
		}
		sendSpeed = true;
	}
}

void JoyStickHandler::sendJoyStickEvent()
{
	try
	{
		if(sendSpeed)
		{
			_base_proxy->setSpeedBase(base_joy_axis.actualY * config.maxAdv , base_joy_axis.actualX * config.maxRot);
			if(fabs(base_joy_axis.actualX) < 0.1 and fabs(base_joy_axis.actualY) < 0.1 )
				sendSpeed = false;
		}
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Fallo la comunicacion a traves del proxy (base). Waiting" << endl;
		cout << "[" << PROGRAM_NAME << "]: Motivo: " << endl << ex << endl;
	}
}

