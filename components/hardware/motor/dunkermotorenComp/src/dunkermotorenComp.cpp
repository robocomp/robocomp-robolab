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
/** \mainpage RoboComp::dunkermotorenComp
 *
 * \section intro_sec Introduction
 *
 * The dunkermotorenComp component...
 *
 * \section interface_sec Interface
 *
 * dunkermotorenComp interface...
 *
 * \section install_sec Installation
 *
 * \subsection install1_ssec Software depencences
 * dunkermotorenComp ...
 *
 * \subsection install2_ssec Compile and install
 * cd dunkermotorenComp
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
 * The configuration file dunkermotorenComp/etc/config ...
 * </p>
 *
 * \subsection execution_ssec Execution
 *
 * Just: "${PATH_TO_BINARY}/dunkermotorenComp --Ice.Config=${PATH_TO_CONFIG_FILE}"
 *
 * \subsection running_ssec Once running
 *
 * ...
 *
 */
#include <QtCore>
#include <QtGui>

#include <Ice/Ice.h>
#include <Ice/Application.h>

#include <rapplication/rapplication.h>

#include "config.h"

#include "worker.h"
#include "monitor.h"

#include "jointmotorI.h"

using namespace std;
using namespace RoboCompJointMotor;


class dunkermotorenComp : public RoboComp::Application
{
private:
	void initialize();
	int numMotors;
	string paramsStr;

public:
	virtual int run(int, char*[]);
};

void dunkermotorenComp::initialize()
{

}

int dunkermotorenComp::run(int argc, char* argv[])
{
#ifdef USE_QTGUI
	QApplication a(argc, argv);  // GUI application
#else
	QCoreApplication a(argc, argv);  // NON-GUI application
#endif
	int status=EXIT_SUCCESS;

	// Remote server proxy access example
	// RemoteComponentPrx remotecomponent_proxy;

	string proxy;

	// User variables

	initialize();


	Worker *worker = new Worker();
	//Monitor thread
	Monitor *monitor = new Monitor(worker, communicator());
	QObject::connect(monitor,SIGNAL(kill()),&a,SLOT(quit()));
	QObject::connect(worker,SIGNAL(kill()),&a,SLOT(quit()));
	monitor->start();
	
	if (!monitor->isRunning())
	{
		qDebug()<<"ERROR: Couldn't start monitor and configuration";
		return status;
	}
	try
	{
		// Server adapter creation and publication
		Ice::ObjectAdapterPtr adapter = communicator()->createObjectAdapter("dunkermotorenComp");
		JointMotorI *jointmotorI = new JointMotorI(worker);
		adapter->add(jointmotorI, communicator()->stringToIdentity("jointmotor"));
		adapter->activate();

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
	dunkermotorenComp app;

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
