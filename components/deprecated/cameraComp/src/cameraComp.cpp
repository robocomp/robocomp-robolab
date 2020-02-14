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
/** \mainpage RoboComp::cameraComp
 *
 * \section intro_sec Introduction
 *
 * The cameraComp component captures images of the robot cameras
 *
 * \section interface_sec Interface
 *
 * cameracomp interface implements functions to obtain images in various formats and configuration parameters associated devices.
 *
 * \section install_sec Installation
 *
 * \subsection install1_ssec Software depencences
 * cameracomp ....
 *
 * \subsection install2_ssec Compile and install
 * cd Components/HAL/cameraComp
 * <br>
 * cmake . && make
 * <br>
 * To install:
 * <br>
 * sudo make install
 *
 * \section guide_sec User guide
 *
 * \subsection config_ssec Configuration file
 *
 * <p>
 * The configuration file cameraComp/etc/config you can select different types of cameras and their settings. See config.
 * </p>
 *
 * \subsection execution_ssec Execution
 *
 * Just: "${PATH_TO_BINARY}/cameraComp --Ice.Config=${PATH_TO_CONFIG_FILE}"
 *
 * \subsection running_ssec Once running
 *
 * ....
 *
 */
#include <QtCore>

#include <Ice/Ice.h>

#include <rapplication/rapplication.h>
#include "config.h"

#ifdef SERVER
#include "cameraI.h"
#endif

#include <JointMotor.h>
#include <DifferentialRobot.h>
#include <Camera.h>

#include <signal.h>

using namespace std;
using namespace RoboComp;

class CameraComp : public RoboComp::Application
{
private:
	RoboCompCamera::TCamParams params;
	void initialize();

public:
	virtual int run(int, char*[]);
};


void sig_term(int);

/**
 * Read configuration parameters from config file and stores them in params structure. Lists of allowed values are defined so incorrect entries can be detected early
 */
void CameraComp::initialize()
{
	QStringList driveList;
	driveList.append(QString("FIREWIRE"));
	driveList.append(QString("V4L2"));
	driveList.append(QString("MPLAYER"));
	driveList.append(QString("GAZEBO"));
	driveList.append(QString("DUMMY"));
	driveList.append(QString("PROSILICA"));
	driveList.append(QString("FLYCAPTURE"));

	QStringList modeList;
	modeList.append(QString("MODE_640x480_YUV422"));
	modeList.append(QString("MODE_640x480_RGB"));
	modeList.append(QString("MODE_640x480_MONO"));
	modeList.append(QString("MODE_640x480_GREY"));
	modeList.append(QString("MODE_320x240_YUV422"));
	modeList.append(QString("MODE_320x240_RGB"));
	modeList.append(QString("MODE_320x240_MONO"));
	modeList.append(QString("MODE_320x240_GREY"));
	modeList.append(QString("MODE_516x338_YUV422"));


	QList< int > widthList;
	widthList.append(160);
	widthList.append(320);
	widthList.append(640);
	widthList.append(514);
	widthList.append(516);


	QList< int > heigthList;
	heigthList.append(120);
	heigthList.append(240);
	heigthList.append(480);
	heigthList.append(388);
	heigthList.append(338);


	QList< int > numCamsList;
	numCamsList.append(1);
	numCamsList.append(2);
	numCamsList.append(3);
	numCamsList.append(4);

	QList< int > focalList;
	focalList.append(440);
	focalList.append(880);

	QList< int > fpsList;
	fpsList.append(15);
	fpsList.append(30);

	QList< int > leftCameraList;
	leftCameraList.append(0);
	leftCameraList.append(1);

	QList< int > rightCameraList;
	rightCameraList.append(0);
	rightCameraList.append(1);

	QList< int > bothCameraList;
	bothCameraList.append(5);

	QList<int> freqList;
	freqList.append(50);
	freqList.append(60);

	// Config file properties
	std::cout << "READING CAMERA PARAMETERS FROM CONFIG FILE" << std::endl;
	configGetString( "Camera.Driver", params.driver , "FIREWIRE", &driveList);
	configGetString( "Camera.Name", params.name , "Apple");
	configGetString( "Camera.Device", params.device , "/dev/video0");//, &deviceList);
	configGetString( "Camera.Mode", params.mode , "MODE_640x480_YUV422", &modeList);
	configGetBool( "Camera.TalkToBaseComp", params.talkToBase , true);
	configGetBool( "Camera.TalkToJointMotorComp", params.talkToJointMotor , true);
	configGetInt( "Camera.Width", params.width, 320, &widthList);
	configGetInt( "Camera.Height", params.height, 240, &heigthList);
	configGetInt( "Camera.NumCameras", params.numCams, 1, &numCamsList);
	configGetInt( "Camera.Focal", params.focal, 440);
	configGetInt( "Camera.FPS", params.FPS, 15, &fpsList);
	configGetInt( "Camera.LeftCamera", params.leftCamera, 0, &leftCameraList);
	configGetInt( "Camera.RightCamera", params.rightCamera, 1, &rightCameraList);
	configGetInt( "Camera.BothCameras", params.bothCameras, 5, &bothCameraList);
	configGetInt( "Camera.Inverted", params.inverted, 0);
	configGetInt( "Camera.Rotated", params.rotated, 0);
	configGetInt( "Camera.LeftInverted", params.leftInverted, 0);
	configGetInt( "Camera.RightInverted", params.rightInverted, 0);
	configGetInt( "Camera.LineFreq", params.lineFreq, 50, &freqList);
	configGetInt( "Camera.Saturation", params.saturation, 50);

	params.size = params.width * params.height;
}

/**
 * Run method of the thread. It is called by the Ice superclass. All application specific processing of the component is carried out here.
 * @param argc number of external params.
 * @param argv[] external params
 * @return exit value
 */
int CameraComp::run(int argc, char* argv[])
{
	QCoreApplication a(argc, argv);  // NON-GUI application
	int status=EXIT_SUCCESS;
	CameraI *cameraI;

	RoboCompJointMotor::JointMotorPrx head_proxy;
	RoboCompDifferentialRobot::DifferentialRobotPrx base_proxy;
	string proxy;
	initialize();

	// Load JointMotor proxy
	try
	{
	  head_proxy = RoboCompJointMotor::JointMotorPrx::uncheckedCast( communicator()->stringToProxy( getProxyString("JointMotorProxy") ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception: " << ex;
		return EXIT_FAILURE;
	}


	// Load Base proxy
	try
	{
		proxy = communicator()->getProperties()->getProperty( "DifferentialRobotProxy" );
		cout << "[" << PROGRAM_NAME << "]: Loading [" << proxy << "] proxy at '" << "DifferentialRobotProxy" << "'..." << endl;
		if( proxy.empty() )
		{
			cout << "[" << PROGRAM_NAME << "]: Error loading proxy config!" << endl;
			return EXIT_FAILURE;
		}

		base_proxy = RoboCompDifferentialRobot::DifferentialRobotPrx::uncheckedCast( communicator()->stringToProxy( proxy ) );
		if( !base_proxy )
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


	// Server initialization
	try
	{
		// Server adapter creation and publication
		Ice::ObjectAdapterPtr adapter = communicator()->createObjectAdapter("CameraComp");
		cameraI = new CameraI(params, head_proxy, base_proxy);
		adapter->add(cameraI, communicator()->stringToIdentity("camera"));
		adapter->activate();

		cout << SERVER_FULL_NAME " started" << endl;

// 		ignoreInterrupt();

		sigset(SIGTERM, sig_term);

		a.exec();
		status = EXIT_SUCCESS;
	}
	catch(const Ice::Exception& ex)
	{
		status = EXIT_FAILURE;
		cout << "[" << PROGRAM_NAME << "]: Exception raised on main thread: " << endl;
		cout << ex;
	}


 	cameraI->terminate();
	communicator()->destroy();

	return status;
}

void sig_term(int)
{
    QCoreApplication::exit();
}

int main(int argc, char* argv[])
{
	bool hasConfig = false;
	string arg;
	CameraComp app;

	// Search in argument list for --Ice.Config= argument
	for (int i = 1; i < argc; ++i)
	{
		arg = argv[i];
		if ( arg.find ( "--Ice.Config=", 0 ) != string::npos )
			hasConfig = true;
	}

	if ( hasConfig )
		app.main( argc, argv );
		//return app.main( argc, argv );
	else
		app.main(argc, argv, "config"); // "config" is the default config file name
		//return app.main(argc, argv, "config"); // "config" is the default config file name

}

