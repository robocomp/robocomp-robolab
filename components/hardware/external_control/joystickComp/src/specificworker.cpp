/*
 *    Copyright (C)2018 by YOUR NAME HERE
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
SpecificWorker::SpecificWorker(MapPrx& mprx, bool startup_check) : GenericWorker(mprx)
{}

void SpecificWorker::initialize(int period)
{
	// Set the base joystick axis initial data
	base_joy_axis.actualX = 0.;
	base_joy_axis.actualY = 0.;

	joystick = new QJoyStick( QString::fromStdString(params["Device"].value));
	jtimer = new QTimer();
	
	// Connect signals
	connect( joystick, SIGNAL( inputEvent(int, int, int) ), this, SLOT(receivedJoyStickEvent(int, int, int) ) );
	connect( jtimer, SIGNAL( timeout() ), this, SLOT( sendJoyStickEvent() ) );
	
	std::cout << PROGRAM_NAME << ": New JoyStick Handler settings: XMotionAxis " << params["XMotionAxis"].value <<" YMotionAxis " << params["YMotionAxis"].value << std::endl;
	
	std::cout << PROGRAM_NAME << ": Max advance speed: "<< params["MaxAdvance"].value <<
	" Max steering speed: " << params["MaxSteering"].value << std::endl;
	
	open();
	//jtimer->start(100);
	timer.stop();
	
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	jtimer->stop();
	joystick->stop();

	disconnect ( joystick );
	disconnect ( jtimer );

	delete jtimer;
	delete joystick;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	this->params = params;
	config.XMotionAxis = std::stoi(params["XMotionAxis"].value);
	config.YMotionAxis = std::stoi(params["YMotionAxis"].value);
	config.SampleRate = std::stoi(params["SampleRate"].value);
	config.maxAdv = std::stoi(params["MaxAdvance"].value);
	config.maxRot = std::stof(params["MaxSteering"].value);
	
	initialize(100);
	return true;
}

void SpecificWorker::compute()
{
}

bool SpecificWorker::open()
{
	if ( joystick->openQJoy() )
	{
		joystick->start();
		if (config.SampleRate < 1) config.SampleRate = 1;
		jtimer->start( 1000 / config.SampleRate );
		
		return true;
	}
	else
		return false;
}

void SpecificWorker::receivedJoyStickEvent(int value, int type, int number)
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
		try
		{
			differentialrobot_proxy->setSpeedBase(base_joy_axis.actualY * config.maxAdv , base_joy_axis.actualX * config.maxRot);
		//	std::cout << base_joy_axis.actualX <<  " " << base_joy_axis.actualY << std::endl;
		}
		catch(const Ice::Exception &ex) { std::cout << ex << std::endl;}
	}
}

void SpecificWorker::sendJoyStickEvent()
{
	try
	{
		if(sendSpeed)
		{
			differentialrobot_proxy->setSpeedBase(base_joy_axis.actualY * config.maxAdv , base_joy_axis.actualX * config.maxRot);
			std::cout << "---------------------sent event to Differential Robot" << std::endl;
			if(fabs(base_joy_axis.actualX) < 0.1 and fabs(base_joy_axis.actualY) < 0.1 )
				sendSpeed = false;
		}
	}
	catch(const Ice::Exception& ex)
	{
		std::cout << "[" << PROGRAM_NAME << "]: Fallo la comunicacion a traves del proxy (base). Waiting" << std::endl;
		std::cout << "[" << PROGRAM_NAME << "]: Motivo: " << endl << ex << std::endl;
	}
}


/////////////////////////////////////////
// ICE pull methods
/////////////////////////////////////////

void SpecificWorker::JoyStick_writeJoyStickBufferedData(const RoboCompJoyStick::JoyStickBufferedData &gbd)
{
	QMutexLocker locker(mutex);
	joystickBufferedData = gbd;
}

void SpecificWorker::JoyStick_readJoyStickBufferedData(RoboCompJoyStick::JoyStickBufferedData &gbd)
{
	QMutexLocker locker(mutex);
	gbd = joystickBufferedData;

}


