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
/** \mainpage RoboComp::genericComp
 *
 * \section intro_sec Introduction
 *
 * The genericComp component...
 *
 * \section interface_sec Interface
 *
 * genericComp interface...
 *
 * \section install_sec Installation
 *
 * \subsection install1_ssec Software depencences
 * genericComp ...
 *
 * \subsection install2_ssec Compile and install
 * cd genericComp
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
 * The configuration file genericComp/etc/specific_config and genericComp/etc/generic_config...
 * </p>
 *
 * \subsection execution_ssec Execution
 *
 * Just: "${PATH_TO_BINARY}/genericComp --Ice.Config=${PATH_TO_CONFIG_FILE}"
 *
 * \subsection running_ssec Once running
 *
 * ...
 *
 */
// QT includes
#include <QtCore>
#include <QtGui>

// ICE includes
#include <Ice/Ice.h>

#include <Ice/Application.h>

#include <rapplication/rapplication.h>
#include <qlog/qlog.h>
// View the config.h file for config options like
// QtGui, etc...
#include "config.h"
#include "genericmonitor.h"
#include "genericworker.h"
#include "specificworker.h"
#include "specificmonitor.h"
#include "commonbehaviorI.h"
#include <commonheadI.h>
#include <muecasheadcontrolI.h>
#include <joystickadapterI.h>

// Includes for remote proxy example
// #include <Remote.h>
#include <ui_guiDlg.h>
#include <Camera.h>
#include <JointMotor.h>
#include <IMU.h>


// User includes here

// Namespaces
using namespace std;
using namespace RoboCompCommonBehavior;
using namespace RoboCompCommonHead;
using namespace RoboCompmuecasheadcontrol;
using namespace RoboCompJoystickAdapter;
using namespace RoboCompCamera;
using namespace RoboCompJointMotor;
using namespace RoboCompIMU;


class muecasheadcontrolComp : public RoboComp::Application
{
private:
	// User private data here

	void initialize();
	MapPrx mprx;

public:
	virtual int run(int, char*[]);
};

void muecasheadcontrolComp::initialize()
{
	// Config file properties read example
	// configGetString( PROPERTY_NAME_1, property1_holder, PROPERTY_1_DEFAULT_VALUE );
	// configGetInt( PROPERTY_NAME_2, property1_holder, PROPERTY_2_DEFAULT_VALUE );
}
int muecasheadcontrolComp::run(int argc, char* argv[])
{
#ifdef USE_QTGUI
	QApplication a(argc, argv);  // GUI application
#else
	QCoreApplication a(argc, argv);  // NON-GUI application
#endif
	int status=EXIT_SUCCESS;

	// Remote server proxy access example
	// RemoteComponentPrx remotecomponent_proxy;
	CameraPrx camera_proxy;
JointMotorPrx jointmotor0_proxy;
JointMotorPrx jointmotor1_proxy;
IMUPrx imu_proxy;


	string proxy;

	// User variables


	initialize();

	// Remote server proxy creation example
	// try
	// {
	// 	// Load the remote server proxy
	//	proxy = getProxyString("RemoteProxy");
	//	remotecomponent_proxy = RemotePrx::uncheckedCast( communicator()->stringToProxy( proxy ) );
	//	if( !remotecomponent_proxy )
	//	{
	//		rInfo(QString("Error loading proxy!"));
	//		return EXIT_FAILURE;
	//	}
	//catch(const Ice::Exception& ex)
	//{
	//	cout << "[" << PROGRAM_NAME << "]: Exception: " << ex << endl;
	//	return EXIT_FAILURE;
	//}
	//rInfo("RemoteProxy initialized Ok!");
	// 	// Now you can use remote server proxy (remotecomponent_proxy) as local object
	//Remote server proxy creation example
	try
	{
		camera_proxy = CameraPrx::uncheckedCast( communicator()->stringToProxy( getProxyString("CameraProxy") ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("CameraProxy initialized Ok!");
	mprx["CameraProxy"] = (::IceProxy::Ice::Object*)(&camera_proxy);//Remote server proxy creation example
	try
	{
		jointmotor0_proxy = JointMotorPrx::uncheckedCast( communicator()->stringToProxy( getProxyString("JointMotor0Proxy") ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("JointMotor0Proxy initialized Ok!");
	mprx["JointMotor0Proxy"] = (::IceProxy::Ice::Object*)(&jointmotor0_proxy);//Remote server proxy creation example
	try
	{
		jointmotor1_proxy = JointMotorPrx::uncheckedCast( communicator()->stringToProxy( getProxyString("JointMotor1Proxy") ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("JointMotor1Proxy initialized Ok!");
	mprx["JointMotor1Proxy"] = (::IceProxy::Ice::Object*)(&jointmotor1_proxy);//Remote server proxy creation example
	try
	{
		imu_proxy = IMUPrx::uncheckedCast( communicator()->stringToProxy( getProxyString("IMUProxy") ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("IMUProxy initialized Ok!");
	mprx["IMUProxy"] = (::IceProxy::Ice::Object*)(&imu_proxy);
	
	GenericWorker *worker = new SpecificWorker(mprx);
	//Monitor thread
	GenericMonitor *monitor = new SpecificMonitor(worker,communicator());
	QObject::connect(monitor,SIGNAL(kill()),&a,SLOT(quit()));
	QObject::connect(worker,SIGNAL(kill()),&a,SLOT(quit()));
	monitor->start();
	
	if ( !monitor->isRunning() )
		return status;
	try
	{
		// Server adapter creation and publication
		Ice::ObjectAdapterPtr adapterCommonBehavior = communicator()->createObjectAdapter("CommonBehavior");
		CommonBehaviorI *commonbehaviorI = new CommonBehaviorI(monitor );
		adapterCommonBehavior->add(commonbehaviorI, communicator()->stringToIdentity("commonbehavior"));
		adapterCommonBehavior->activate();
		// Server adapter creation and publication
		Ice::ObjectAdapterPtr adapterCommonHead = communicator()->createObjectAdapter("CommonHeadComp");
		CommonHeadI *commonhead = new CommonHeadI(worker);
		adapterCommonHead->add(commonhead, communicator()->stringToIdentity("commonhead"));

		adapterCommonHead->activate();
		Ice::ObjectAdapterPtr adaptermuecasheadcontrol = communicator()->createObjectAdapter("muecasheadcontrolComp");
		muecasheadcontrolI *muecasheadcontrol = new muecasheadcontrolI(worker);
		adaptermuecasheadcontrol->add(muecasheadcontrol, communicator()->stringToIdentity("muecasheadcontrol"));

		adaptermuecasheadcontrol->activate();
		Ice::ObjectAdapterPtr adapterJoystickAdapter = communicator()->createObjectAdapter("JoystickAdapterComp");
		JoystickAdapterI *joystickadapter = new JoystickAdapterI(worker);
		adapterJoystickAdapter->add(joystickadapter, communicator()->stringToIdentity("joystickadapter"));

		adapterJoystickAdapter->activate();
		cout << SERVER_FULL_NAME " started" << endl;

		// User defined QtGui elements ( main window, dialogs, etc )

#ifdef USE_QTGUI
		//ignoreInterrupt(); // Uncomment if you want the component to ignore console SIGINT signal (ctrl+c).
		a.setQuitOnLastWindowClosed( true );
#endif
		// Run QT Application Event Loop
		a.exec();
		status = EXIT_SUCCESS;
	}
	catch(const Ice::Exception& ex)
	{
		status = EXIT_FAILURE;

		cout << "[" << PROGRAM_NAME << "]: Exception raised on main thread: " << endl;
		cout << ex;

#ifdef USE_QTGUI
		a.quit();
#endif
	}

	return status;
}

int main(int argc, char* argv[])
{
	bool hasConfig = false;
	string arg;
	muecasheadcontrolComp app;

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
		return app.main(argc, argv, "../etc/generic_config"); // "config" is the default config file name
}