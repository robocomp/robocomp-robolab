/*
 *    Copyright (C) 2016 by YOUR NAME HERE
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


/** \mainpage RoboComp::stableOdometry
 *
 * \section intro_sec Introduction
 *
 * The stableOdometry component...
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
 * cd stableOdometry
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
 * Just: "${PATH_TO_BINARY}/stableOdometry --Ice.Config=${PATH_TO_CONFIG_FILE}"
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
#include <qlog/qlog.h>

#include "config.h"
#include "genericmonitor.h"
#include "genericworker.h"
#include "specificworker.h"
#include "specificmonitor.h"
#include "commonbehaviorI.h"

#include <aprilbasedlocalizationI.h>
#include <cgrtopicI.h>

#include <OmniRobot.h>
#include <AprilBasedLocalization.h>
#include <CGR.h>


// User includes here

// Namespaces
using namespace std;
using namespace RoboCompCommonBehavior;

using namespace RoboCompOmniRobot;
using namespace RoboCompAprilBasedLocalization;
using namespace RoboCompCGR;



class stableOdometry : public RoboComp::Application
{
public:
	stableOdometry (QString prfx) { prefix = prfx.toStdString(); }
private:
	void initialize();
	std::string prefix;
	MapPrx mprx;

public:
	virtual int run(int, char*[]);
};

void ::stableOdometry::initialize()
{
	// Config file properties read example
	// configGetString( PROPERTY_NAME_1, property1_holder, PROPERTY_1_DEFAULT_VALUE );
	// configGetInt( PROPERTY_NAME_2, property1_holder, PROPERTY_2_DEFAULT_VALUE );
}

int ::stableOdometry::run(int argc, char* argv[])
{
	QCoreApplication a(argc, argv);  // NON-GUI application


	sigset_t sigs;
	sigemptyset(&sigs);
	sigaddset(&sigs, SIGHUP);
	sigaddset(&sigs, SIGINT);
	sigaddset(&sigs, SIGTERM);
	sigprocmask(SIG_UNBLOCK, &sigs, 0);



	int status=EXIT_SUCCESS;

	OmniRobotPrx omnirobot_proxy;
	CGRPrx cgr_proxy;

	string proxy, tmp;
	initialize();


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "OmniRobotProxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy OmniRobotProxy\n";
		}
		omnirobot_proxy = OmniRobotPrx::uncheckedCast( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("OmniRobotProxy initialized Ok!");
	mprx["OmniRobotProxy"] = (::IceProxy::Ice::Object*)(&omnirobot_proxy);//Remote server proxy creation example


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "CGRProxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy CGRProxy\n";
		}
		cgr_proxy = CGRPrx::uncheckedCast( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("CGRProxy initialized Ok!");
	mprx["CGRProxy"] = (::IceProxy::Ice::Object*)(&cgr_proxy);//Remote server proxy creation example

	IceStorm::TopicManagerPrx topicManager = IceStorm::TopicManagerPrx::checkedCast(communicator()->propertyToProxy("TopicManager.Proxy"));


	SpecificWorker *worker = new SpecificWorker(mprx);
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
		// Server adapter creation and publication
		if (not GenericMonitor::configGetString(communicator(), prefix, "CommonBehavior.Endpoints", tmp, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy CommonBehavior\n";
		}
		Ice::ObjectAdapterPtr adapterCommonBehavior = communicator()->createObjectAdapterWithEndpoints("commonbehavior", tmp);
		CommonBehaviorI *commonbehaviorI = new CommonBehaviorI(monitor );
		adapterCommonBehavior->add(commonbehaviorI, communicator()->stringToIdentity("commonbehavior"));
		adapterCommonBehavior->activate();







		// Server adapter creation and publication
		if (not GenericMonitor::configGetString(communicator(), prefix, "AprilBasedLocalizationTopic.Endpoints", tmp, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy AprilBasedLocalizationProxy";
		}
		Ice::ObjectAdapterPtr AprilBasedLocalization_adapter = communicator()->createObjectAdapterWithEndpoints("aprilbasedlocalization", tmp);
		AprilBasedLocalizationPtr aprilbasedlocalizationI_ = new AprilBasedLocalizationI(worker);
		Ice::ObjectPrx aprilbasedlocalization = AprilBasedLocalization_adapter->addWithUUID(aprilbasedlocalizationI_)->ice_oneway();
		IceStorm::TopicPrx aprilbasedlocalization_topic;
		if(!aprilbasedlocalization_topic){
		try {
			aprilbasedlocalization_topic = topicManager->create("AprilBasedLocalization");
		}
		catch (const IceStorm::TopicExists&) {
		//Another client created the topic
		try{
			aprilbasedlocalization_topic = topicManager->retrieve("AprilBasedLocalization");
		}
		catch(const IceStorm::NoSuchTopic&)
		{
			//Error. Topic does not exist
			}
		}
		IceStorm::QoS qos;
		aprilbasedlocalization_topic->subscribeAndGetPublisher(qos, aprilbasedlocalization);
		}
		AprilBasedLocalization_adapter->activate();

		// Server adapter creation and publication
		if (not GenericMonitor::configGetString(communicator(), prefix, "CGRTopicTopic.Endpoints", tmp, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy CGRTopicProxy";
		}
		Ice::ObjectAdapterPtr CGRTopic_adapter = communicator()->createObjectAdapterWithEndpoints("cgrtopic", tmp);
		CGRTopicPtr cgrtopicI_ = new CGRTopicI(worker);
		Ice::ObjectPrx cgrtopic = CGRTopic_adapter->addWithUUID(cgrtopicI_)->ice_oneway();
		IceStorm::TopicPrx cgrtopic_topic;
		if(!cgrtopic_topic){
		try {
			cgrtopic_topic = topicManager->create("CGRTopic");
		}
		catch (const IceStorm::TopicExists&) {
		//Another client created the topic
		try{
			cgrtopic_topic = topicManager->retrieve("CGRTopic");
		}
		catch(const IceStorm::NoSuchTopic&)
		{
			//Error. Topic does not exist
			}
		}
		IceStorm::QoS qos;
		cgrtopic_topic->subscribeAndGetPublisher(qos, cgrtopic);
		}
		CGRTopic_adapter->activate();

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

#ifdef USE_QTGUI
		a.quit();
#endif
		monitor->exit(0);
}

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
	::stableOdometry app(prefix);

	return app.main(argc, argv, configFile.c_str());
}

