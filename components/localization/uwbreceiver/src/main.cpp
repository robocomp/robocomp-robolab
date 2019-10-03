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


/** \mainpage RoboComp::uwbreceiver
 *
 * \section intro_sec Introduction
 *
 * The uwbreceiver component...
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
 * cd uwbreceiver
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
 * Just: "${PATH_TO_BINARY}/uwbreceiver --Ice.Config=${PATH_TO_CONFIG_FILE}"
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

#include <fullposeestimationI.h>

#include <FullPoseEstimation.h>


// User includes here

// Namespaces
using namespace std;
using namespace RoboCompCommonBehavior;

class uwbreceiver : public RoboComp::Application
{
public:
	uwbreceiver (QString prfx) { prefix = prfx.toStdString(); }
private:
	void initialize();
	std::string prefix;
	TuplePrx tprx;

public:
	virtual int run(int, char*[]);
};

void ::uwbreceiver::initialize()
{
	// Config file properties read example
	// configGetString( PROPERTY_NAME_1, property1_holder, PROPERTY_1_DEFAULT_VALUE );
	// configGetInt( PROPERTY_NAME_2, property1_holder, PROPERTY_2_DEFAULT_VALUE );
}

int ::uwbreceiver::run(int argc, char* argv[])
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

	FullPoseEstimationPubPrxPtr fullposeestimationpub_pubproxy;

	string proxy, tmp;
	initialize();

	IceStorm::TopicManagerPrxPtr topicManager;
	try
	{
		topicManager = Ice::checkedCast<IceStorm::TopicManagerPrx>(communicator()->propertyToProxy("TopicManager.Proxy"));
	}
	catch (const Ice::Exception &ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception: STORM not running: " << ex << endl;
		return EXIT_FAILURE;
	}
	std::shared_ptr<IceStorm::TopicPrx> fullposeestimationpub_topic;

	while (!fullposeestimationpub_topic)
	{
		try
		{
			fullposeestimationpub_topic = topicManager->retrieve("FullPoseEstimationPub");
		}
		catch (const IceStorm::NoSuchTopic&)
		{
			cout << "[" << PROGRAM_NAME << "]: ERROR retrieving FullPoseEstimationPub topic. \n";
			try
			{
				fullposeestimationpub_topic = topicManager->create("FullPoseEstimationPub");
			}
			catch (const IceStorm::TopicExists&){
				// Another client created the topic.
				cout << "[" << PROGRAM_NAME << "]: ERROR publishing the FullPoseEstimationPub topic. It's possible that other component have created\n";
			}
		}
	}

	auto fullposeestimationpub_pub = fullposeestimationpub_topic->getPublisher()->ice_oneway();
	fullposeestimationpub_pubproxy = Ice::uncheckedCast<FullPoseEstimationPubPrx>(fullposeestimationpub_pub);

	tprx = std::make_tuple(fullposeestimationpub_pubproxy);
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
			if (not GenericMonitor::configGetString(communicator(), prefix, "FullPoseEstimation.Endpoints", tmp, ""))
			{
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy FullPoseEstimation";
			}
			Ice::ObjectAdapterPtr adapterFullPoseEstimation = communicator()->createObjectAdapterWithEndpoints("FullPoseEstimation", tmp);
			auto fullposeestimation = std::make_shared<FullPoseEstimationI>(worker);
			adapterFullPoseEstimation->add(fullposeestimation, Ice::stringToIdentity("fullposeestimation"));
			adapterFullPoseEstimation->activate();
			cout << "[" << PROGRAM_NAME << "]: FullPoseEstimation adapter created in port " << tmp << endl;
			}
			catch (const IceStorm::TopicExists&){
				cout << "[" << PROGRAM_NAME << "]: ERROR creating or activating adapter for FullPoseEstimation\n";
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
	::uwbreceiver app(prefix);

	return app.main(argc, argv, configFile.c_str());
}
