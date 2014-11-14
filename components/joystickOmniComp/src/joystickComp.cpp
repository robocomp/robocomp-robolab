
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
/** \mainpage RoboComp::jostickComp
 *
 * \section intro_sec Introduction
 *
 * CammotionComp is a component to send joystikc movements to omniRobotComp. Basically read joystick movements, transform then to speed orders and send to omniRobotComp. It's a simple way to guide the robot.
 *
 * \section interface_sec Interface
 *
 * 
 *
 * \section install_sec Installation
 *
 * \subsection install1_ssec Software depencences
 * cammotionComp does not have specific software dependencies.
 *
 * \subsection install2_ssec Compile
 * cd Components/Robolab/Stable/cammotionComp
 * <br>
 * cmake . && make
 * <br>
 * To install:
 * sudo make install
 *
 * \subsection install2_ssec Compile and install
 * cd Components/Robolab/Stable/joystickComp
 *
 *
 * \section guide_sec User guide
 *
 *
 *
 * \subsection config_ssec Configuration file
 *
 * 
 *
 * \subsection execution_ssec Execution
 *
 * Just: "${PATH_TO_BINARY}/joystickComp --Ice.Config=${PATH_TO_CONFIG_FILE}"
 *
 * \subsection running_ssec Once running
 *
 * 
 *
 */



#include <Ice/Ice.h>
#include <Ice/Application.h>
#include <IceUtil/IceUtil.h>

#include <QtCore/QtCore>
#include <QtCore/QCoreApplication>

// Interface implementation
#include <joystickI.h>

// Includes for remote proxy example
#include <OmniRobot.h>
#include <JoyStick.h>
#include <joystickhandler.h>
#include <rapplication/rapplication.h>

#include <const.h>

using namespace std;

class JoyStickComp : public RoboComp::Application
{
private:
	// User private data here
	string joystick_device;
	JoyStickHandler::qjh_cfg_t joy_config;

	void initialize();

public:
	virtual int run(int, char*[]);
};

void JoyStickComp::initialize()
{
	configGetString( "Joy.Device", joystick_device, "/dev/input/js0" );
	configGetInt( "Joy.XMotionAxis", joy_config.XMotionAxis, 0 );
	configGetInt( "Joy.YMotionAxis", joy_config.YMotionAxis, 1 );
	configGetInt( "Joy.ZMotionAxis", joy_config.ZMotionAxis, 2 );
	configGetInt( "Joy.SampleRate", joy_config.SampleRate,5);
	configGetInt( "Joy.MaxAdvanceX", joy_config.maxAdvX, 120 );
	configGetInt( "Joy.MaxAdvanceZ", joy_config.maxAdvZ, 120 );
	configGetFloat( "Joy.MaxSteering", joy_config.maxRot, 1.0 );
}

int main(int argc, char* argv[])
{
	bool hasConfig = false;
	string arg;
	JoyStickComp app;
	
	// Search in argument list for --Ice.Config= argument
	for (int i = 1; i < argc; ++i)
	{
		arg = argv[i];
		if ( arg.find ( "--Ice.Config=", 0 ) != string::npos ) 
			hasConfig = true;
	}
	
	if ( hasConfig )
		return app.main( argc, argv );
	else
		return app.main(argc, argv, "config"); // "config" is the default config file name
}

int JoyStickComp::run(int argc, char* argv[])
{
	QCoreApplication a(argc, argv);  //CHANGE TO QApplication for GUI based components !!!
	int status=EXIT_SUCCESS;
	
	JoyStickHandler *jh;
		
	// Remote server proxy access example
	string proxy;
	RoboCompOmniRobot::OmniRobotPrx base_proxy;
 
	initialize();

	try
	{
		// Load the base server proxies
		proxy = communicator()->getProperties()->getProperty( DEFAULT_BASE_SERVER_PROXY_STRING );
		cout << "[" << PROGRAM_NAME << "]: Loading [" << proxy << "] proxy at '" << DEFAULT_BASE_SERVER_PROXY_STRING << "'..." << endl;
		if( proxy.empty() ) 
		{
			cout << "[" << PROGRAM_NAME << "]: Error loading proxy config!" << endl;
			return EXIT_FAILURE;
		}

		base_proxy = RoboCompOmniRobot::OmniRobotPrx::uncheckedCast( communicator()->stringToProxy( proxy ) );
		if (!base_proxy)
		{
			cout << "[" << PROGRAM_NAME << "]: Error loading proxy!" << endl;
			return EXIT_FAILURE;
		}
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception: " << ex;
		return EXIT_FAILURE;
	}

	try
	{
		// Set up joystick
		jh = new JoyStickHandler(joy_config, base_proxy, joystick_device.c_str() );
		if (!jh->open())
		{
			cout << "[" << PROGRAM_NAME << "]: Unable to open device: " << joystick_device << endl;
			return EXIT_FAILURE;
		}
		
	// Server adapter creation and publication
		Ice::ObjectAdapterPtr adapter = communicator()->createObjectAdapter("JoyStickComp");
		JoyStickI *joystickI = new JoyStickI();
		adapter->add(joystickI, communicator()->stringToIdentity("joystick"));
		adapter->activate();
	
 		cout << "RobolabMod::JoyStick started" << endl;
		a.exec();

		status = EXIT_SUCCESS;
	}
	catch(const Ice::Exception& ex)
	{
		status = EXIT_FAILURE;
	}
	
	return status;
}
