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
#include <getapriltagsI.h>

#include <Camera.h>
#include <RGBD.h>
#include <RGBDBus.h>
#include <AprilTags.h>


using namespace std;
using namespace RoboCompCommonBehavior;
using namespace RoboCompGetAprilTags;
using namespace RoboCompCamera;
using namespace RoboCompRGBD;
using namespace RoboCompRGBDBus;
using namespace RoboCompAprilTags;


class AprilTagsComp : public RoboComp::Application
{
private:
	void initialize();
	MapPrx mprx;

public:
	virtual int run(int, char*[]);
};

void AprilTagsComp::initialize()
{
}


int AprilTagsComp::run(int argc, char* argv[])
{
#ifdef USE_QTGUI
	QApplication a(argc, argv);  // GUI application
#else
	QCoreApplication a(argc, argv);  // NON-GUI application
#endif
	int status=EXIT_SUCCESS;

	CameraPrx camera_proxy;
	RGBDPrx rgbd_proxy;
	RGBDBusPrx rgbdbus_proxy;
	AprilTagsPrx apriltags_proxy;


	string proxy;

	initialize();

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
		rgbd_proxy = RGBDPrx::uncheckedCast( communicator()->stringToProxy( getProxyString("RGBDProxy") ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception: " << ex;
		return EXIT_FAILURE;
	}

	rInfo("RGBDProxy initialized Ok!");
	mprx["RGBDProxy"] = (::IceProxy::Ice::Object*)(&rgbd_proxy);//Remote server proxy creation example
	try
	{
		rgbdbus_proxy = RGBDBusPrx::uncheckedCast( communicator()->stringToProxy( getProxyString("RGBDBusProxy") ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("RGBDBusProxy initialized Ok!");
	mprx["RGBDBusProxy"] = (::IceProxy::Ice::Object*)(&rgbdbus_proxy);


	IceStorm::TopicManagerPrx topicManager;

	while (true)
	{
		try
		{
			topicManager = IceStorm::TopicManagerPrx::checkedCast(communicator()->propertyToProxy("TopicManager.Proxy"));
			break;
		}
		catch(...)
		{
			printf("TopicManager.Proxy not available yet. Sleeping...\n");
			sleep(1);
		}
	}
	
	IceStorm::TopicPrx apriltags_topic;

	while (!apriltags_topic)
	{
		try
		{
			apriltags_topic = topicManager->retrieve("AprilTags");
		}
		catch (const IceStorm::NoSuchTopic&)
		{
			try
			{
				apriltags_topic = topicManager->create("AprilTags");
			}
			catch (const IceStorm::TopicExists&){
				// Another client created the topic.
			}
		}
		printf("Topic not available yet. Sleeping...\n");
		sleep(1);
	}

	Ice::ObjectPrx apriltags_pub = apriltags_topic->getPublisher()->ice_oneway();
	AprilTagsPrx apriltags = AprilTagsPrx::uncheckedCast(apriltags_pub);
	mprx["AprilTagsPub"] = (::IceProxy::Ice::Object*)(&apriltags);
	GenericWorker *worker = new SpecificWorker(mprx);
	
	//Monitor thread
	GenericMonitor *monitor = new SpecificMonitor(worker,communicator());
	QObject::connect(monitor, SIGNAL(kill()), &a, SLOT(quit()));
	QObject::connect(worker, SIGNAL(kill()), &a, SLOT(quit()));
	monitor->start();
	worker->setPeriod(100);
	
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
		Ice::ObjectAdapterPtr adapterGetAprilTags = communicator()->createObjectAdapter("GetAprilTagsComp");
		GetAprilTagsI *getapriltags = new GetAprilTagsI(worker);
		adapterGetAprilTags->add(getapriltags, communicator()->stringToIdentity("getapriltags"));

		adapterGetAprilTags->activate();
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
	bool hasConfig = false;
	string arg;
	AprilTagsComp app;

	// Search in argument list for --Ice.Config= argument
	for (int i = 1; i < argc; ++i)
	{
		arg = argv[i];
		if (arg.find ( "--Ice.Config=", 0 ) != string::npos)
			hasConfig = true;
	}

	if ( hasConfig )
		return app.main( argc, argv );
	else
		return app.main(argc, argv, "../etc/generic_config"); // "config" is the default config file name
}

