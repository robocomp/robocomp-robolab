/*
 *    Copyright (C) 2019 by YOUR NAME HERE
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


<<<<<<< HEAD
/** \mainpage RoboComp::cameraipcpp
 *
 * \section intro_sec Introduction
 *
 * The cameraipcpp component...
=======
/** \mainpage RoboComp::cameraipccp
 *
 * \section intro_sec Introduction
 *
 * The cameraipccp component...
>>>>>>> eea2392695743c234d5db54b3b0e434d726fbb1b
 *
 * \section interface_sec Interface
 *
 * interface...
 *
 * \section install_sec Installation
 *
 * \subsection install1_ssec Software depencences
 * ...
 *
 * \subsection install2_ssec Compile and install
<<<<<<< HEAD
 * cd cameraipcpp
=======
 * cd cameraipccp
>>>>>>> eea2392695743c234d5db54b3b0e434d726fbb1b
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
 * The configuration file etc/config...
 * </p>
 *
 * \subsection execution_ssec Execution
 *
<<<<<<< HEAD
 * Just: "${PATH_TO_BINARY}/cameraipcpp --Ice.Config=${PATH_TO_CONFIG_FILE}"
=======
 * Just: "${PATH_TO_BINARY}/cameraipccp --Ice.Config=${PATH_TO_CONFIG_FILE}"
>>>>>>> eea2392695743c234d5db54b3b0e434d726fbb1b
 *
 * \subsection running_ssec Once running
 *
 * ...
 *
 */
#include <signal.h>

// QT includes
#include <QtCore>
#include <QtGui>

// ICE includes
#include <Ice/Ice.h>
#include <IceStorm/IceStorm.h>
#include <Ice/Application.h>

#include <rapplication/rapplication.h>
#include <sigwatch/sigwatch.h>
#include <qlog/qlog.h>

#include "config.h"
#include "genericmonitor.h"
#include "genericworker.h"
#include "specificworker.h"
#include "specificmonitor.h"
#include "commonbehaviorI.h"

#include <camerasimpleI.h>

#include <CameraSimple.h>
<<<<<<< HEAD
=======
#include <GetAprilTags.h>
>>>>>>> eea2392695743c234d5db54b3b0e434d726fbb1b


// User includes here

// Namespaces
using namespace std;
using namespace RoboCompCommonBehavior;

<<<<<<< HEAD
class cameraipcpp : public RoboComp::Application
{
public:
	cameraipcpp (QString prfx) { prefix = prfx.toStdString(); }
=======
class cameraipccp : public RoboComp::Application
{
public:
	cameraipccp (QString prfx) { prefix = prfx.toStdString(); }
>>>>>>> eea2392695743c234d5db54b3b0e434d726fbb1b
private:
	void initialize();
	std::string prefix;
	TuplePrx tprx;

public:
	virtual int run(int, char*[]);
};

<<<<<<< HEAD
void ::cameraipcpp::initialize()
=======
void ::cameraipccp::initialize()
>>>>>>> eea2392695743c234d5db54b3b0e434d726fbb1b
{
	// Config file properties read example
	// configGetString( PROPERTY_NAME_1, property1_holder, PROPERTY_1_DEFAULT_VALUE );
	// configGetInt( PROPERTY_NAME_2, property1_holder, PROPERTY_2_DEFAULT_VALUE );
}

<<<<<<< HEAD
int ::cameraipcpp::run(int argc, char* argv[])
=======
int ::cameraipccp::run(int argc, char* argv[])
>>>>>>> eea2392695743c234d5db54b3b0e434d726fbb1b
{
	QCoreApplication a(argc, argv);  // NON-GUI application


	sigset_t sigs;
	sigemptyset(&sigs);
	sigaddset(&sigs, SIGHUP);
	sigaddset(&sigs, SIGINT);
	sigaddset(&sigs, SIGTERM);
	sigprocmask(SIG_UNBLOCK, &sigs, 0);

	UnixSignalWatcher sigwatch;
	sigwatch.watchForSignal(SIGINT);
	sigwatch.watchForSignal(SIGTERM);
	QObject::connect(&sigwatch, SIGNAL(unixSignal(int)), &a, SLOT(quit()));

	int status=EXIT_SUCCESS;

<<<<<<< HEAD

=======
	GetAprilTagsPrxPtr getapriltags_proxy;
>>>>>>> eea2392695743c234d5db54b3b0e434d726fbb1b
	string proxy, tmp;
	initialize();


<<<<<<< HEAD
	tprx = std::tuple<>();
=======
	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "GetAprilTagsProxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy GetAprilTagsProxy\n";
		}
		getapriltags_proxy = Ice::uncheckedCast<GetAprilTagsPrx>( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy GetAprilTags: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("GetAprilTagsProxy initialized Ok!");


	tprx = std::make_tuple(getapriltags_proxy);
>>>>>>> eea2392695743c234d5db54b3b0e434d726fbb1b
	SpecificWorker *worker = new SpecificWorker(tprx);
	//Monitor thread
	SpecificMonitor *monitor = new SpecificMonitor(worker,communicator());
	QObject::connect(monitor, SIGNAL(kill()), &a, SLOT(quit()));
	QObject::connect(worker, SIGNAL(kill()), &a, SLOT(quit()));
	monitor->start();

	if ( !monitor->isRunning() )
		return status;

	while (!monitor->ready)
	{
		usleep(10000);
	}

	try
	{
		try {
			// Server adapter creation and publication
			if (not GenericMonitor::configGetString(communicator(), prefix, "CommonBehavior.Endpoints", tmp, "")) {
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy CommonBehavior\n";
			}
			Ice::ObjectAdapterPtr adapterCommonBehavior = communicator()->createObjectAdapterWithEndpoints("commonbehavior", tmp);
			auto commonbehaviorI = std::make_shared<CommonBehaviorI>(monitor);
			adapterCommonBehavior->add(commonbehaviorI, Ice::stringToIdentity("commonbehavior"));
			adapterCommonBehavior->activate();
		}
		catch(const Ice::Exception& ex)
		{
			status = EXIT_FAILURE;

			cout << "[" << PROGRAM_NAME << "]: Exception raised while creating CommonBehavior adapter: " << endl;
			cout << ex;

		}



		try
		{
			// Server adapter creation and publication
			if (not GenericMonitor::configGetString(communicator(), prefix, "CameraSimple.Endpoints", tmp, ""))
			{
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy CameraSimple";
			}
			Ice::ObjectAdapterPtr adapterCameraSimple = communicator()->createObjectAdapterWithEndpoints("CameraSimple", tmp);
			auto camerasimple = std::make_shared<CameraSimpleI>(worker);
			adapterCameraSimple->add(camerasimple, Ice::stringToIdentity("camerasimple"));
			adapterCameraSimple->activate();
			cout << "[" << PROGRAM_NAME << "]: CameraSimple adapter created in port " << tmp << endl;
			}
			catch (const IceStorm::TopicExists&){
				cout << "[" << PROGRAM_NAME << "]: ERROR creating or activating adapter for CameraSimple\n";
			}



		// Server adapter creation and publication
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

	}
	#ifdef USE_QTGUI
		a.quit();
	#endif

	status = EXIT_SUCCESS;
	monitor->terminate();
	monitor->wait();
	delete worker;
	delete monitor;
	return status;
}

int main(int argc, char* argv[])
{
	string arg;

	// Set config file
	std::string configFile = "config";
	if (argc > 1)
	{
		std::string initIC("--Ice.Config=");
		size_t pos = std::string(argv[1]).find(initIC);
		if (pos == 0)
		{
			configFile = std::string(argv[1]+initIC.size());
		}
		else
		{
			configFile = std::string(argv[1]);
		}
	}

	// Search in argument list for --prefix= argument (if exist)
	QString prefix("");
	QString prfx = QString("--prefix=");
	for (int i = 2; i < argc; ++i)
	{
		arg = argv[i];
		if (arg.find(prfx.toStdString(), 0) == 0)
		{
			prefix = QString::fromStdString(arg).remove(0, prfx.size());
			if (prefix.size()>0)
				prefix += QString(".");
			printf("Configuration prefix: <%s>\n", prefix.toStdString().c_str());
		}
	}
<<<<<<< HEAD
	::cameraipcpp app(prefix);
=======
	::cameraipccp app(prefix);
>>>>>>> eea2392695743c234d5db54b3b0e434d726fbb1b

	return app.main(argc, argv, configFile.c_str());
}
