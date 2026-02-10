/*
 *    Copyright (C) 2025 by YOUR NAME HERE
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


/** \mainpage RoboComp::mapping_component
 *
 * \section intro_sec Introduction
 *
 * The mapping_component component...
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
 * cd mapping_component
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
 * Just: "${PATH_TO_BINARY}/mapping_component --Ice.Config=${PATH_TO_CONFIG_FILE}"
 *
 * \subsection running_ssec Once running
 *
 * ...
 *
 */
#include <signal.h>

// QT includes
#include <QtCore>
#include <QtWidgets>

// ICE includes
#include <Ice/Ice.h>
#include <IceStorm/IceStorm.h>
#include <Ice/Application.h>

#include <ConfigLoader/ConfigLoader.h>

#include <sigwatch/sigwatch.h>

#include "genericworker.h"
#include "../src/specificworker.h"

#include <fullposeestimationpubI.h>

#include <DifferentialRobot.h>
#include <FullPoseEstimation.h>
#include <FullPoseEstimationPub.h>
#include <GenericBase.h>
#include <Lidar3D.h>
#include <OmniRobot.h>

//#define USE_QTGUI

#define PROGRAM_NAME    "mapping_component"
#define SERVER_FULL_NAME   "RoboComp mapping_component::mapping_component"


template <typename ProxyType, typename ProxyPointer>
void require(const Ice::CommunicatorPtr& communicator,
             const std::string& proxyConfig, 
             const std::string& proxyName,
             ProxyPointer& proxy)
{
    try
    {
        proxy = Ice::uncheckedCast<ProxyType>(communicator->stringToProxy(proxyConfig));
        std::cout << proxyName << " initialized Ok!\n";
    }
    catch(const Ice::Exception& ex)
    {
        std::cout << "[" << PROGRAM_NAME << "]: Exception creating proxy " << proxyName << ": " << ex;
        throw;
    }
}

template <typename SubInterfaceType>
void subscribe( const Ice::CommunicatorPtr& communicator,
                const IceStorm::TopicManagerPrxPtr& topicManager,
                const std::string& endpointConfig,
                std::string name_topic,
                const std::string& topicBaseName,
                SpecificWorker* worker,
                int index,
                std::shared_ptr<IceStorm::TopicPrx> topic,
                Ice::ObjectPrxPtr& proxy, 
                const std::string& programName)
{
    try   
    {  
        if (!name_topic.empty()) name_topic += "/";
        name_topic += topicBaseName;

        Ice::ObjectAdapterPtr adapter = communicator->createObjectAdapterWithEndpoints(name_topic, endpointConfig);
        auto servant = std::make_shared<SubInterfaceType>(worker, index);
        auto proxy = adapter->addWithUUID(servant)->ice_oneway();

        std::cout << "[\033[1;36m" << programName << "\033[0m]: \033[32mINFO\033[0m Topic: " 
                  << name_topic << " will be used in subscription. \033[0m\n";

        std::shared_ptr<IceStorm::TopicPrx> topic;
        if(!topic)
        {
            try {
                topic = topicManager->create(name_topic);
                std::cout << "\n\n[\033[1;36m" << programName << "\033[0m]: \033[1;33mWARNING\033[0m " 
                          << name_topic << " topic did not create. \033[32mTopic created\033[0m\n\n";
            }
            catch (const IceStorm::TopicExists&) {
                try{
                    std::cout << "[\033[31m" << programName << "\033[0m]: \033[1;33mWARNING\033[0m Probably other client already opened the topic. \033[32mTrying to connect.\033[0m\n";
                    topic = topicManager->retrieve(name_topic);
                }
                catch(const IceStorm::NoSuchTopic&)
                {
                    std::cout << "[" << programName << "]: Topic doesn't exists and couldn't be created.\n";
                    return;
                }
            }
            catch(const IceUtil::NullHandleException&)
            {
                std::cout << "[\033[31m" << programName << "\033[0m]: \033[31mERROR\033[0m TopicManager is Null.\n";
                throw;
            }
            IceStorm::QoS qos;
            topic->subscribeAndGetPublisher(qos, proxy);
        }
        adapter->activate();
    }
    catch(const IceStorm::NoSuchTopic&)
    {
        std::cout << "[" << PROGRAM_NAME << "]: Error creating topic.\n";
    }
}


class mapping_component : public Ice::Application
{
public:
	mapping_component (QString configFile, QString prfx, bool startup_check) { 
		this->configFile = configFile.toStdString();
		this->prefix = prfx.toStdString();
		this->startup_check_flag=startup_check; 

		initialize();
		}

	Ice::InitializationData getInitializationDataIce();

private:
	void initialize();
	std::string prefix, configFile;
	ConfigLoader configLoader;
	TuplePrx tprx;
	bool startup_check_flag = false;

public:
	virtual int run(int, char*[]);
};

Ice::InitializationData mapping_component::getInitializationDataIce(){
        Ice::InitializationData initData;
        initData.properties = Ice::createProperties();
        initData.properties->setProperty("Ice.Warn.Connections", this->configLoader.get<std::string>("Ice.Warn.Connections"));
        initData.properties->setProperty("Ice.Trace.Network", this->configLoader.get<std::string>("Ice.Trace.Network"));
        initData.properties->setProperty("Ice.Trace.Protocol", this->configLoader.get<std::string>("Ice.Trace.Protocol"));
        initData.properties->setProperty("Ice.MessageSizeMax", this->configLoader.get<std::string>("Ice.MessageSizeMax"));
		return initData;
}

void mapping_component::initialize()
{
    this->configLoader.load(this->configFile);
	this->configLoader.printConfig();
	std::cout<<std::endl;
}

int mapping_component::run(int argc, char* argv[])
{
#ifdef USE_QTGUI
	QApplication a(argc, argv);  // GUI application
#else
	QCoreApplication a(argc, argv);  // NON-GUI application
#endif


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

	std::shared_ptr<IceStorm::TopicPrx> fullposeestimationpub_topic;
	Ice::ObjectPrxPtr fullposeestimationpub;

	RoboCompDifferentialRobot::DifferentialRobotPrxPtr differentialrobot_proxy;
	RoboCompLidar3D::Lidar3DPrxPtr lidar3d_proxy;
	RoboCompOmniRobot::OmniRobotPrxPtr omnirobot_proxy;


	//Require code
	require<RoboCompDifferentialRobot::DifferentialRobotPrx, RoboCompDifferentialRobot::DifferentialRobotPrxPtr>(communicator(),
	                    configLoader.get<std::string>("Proxies.DifferentialRobot"), "DifferentialRobotProxy", differentialrobot_proxy);
	require<RoboCompLidar3D::Lidar3DPrx, RoboCompLidar3D::Lidar3DPrxPtr>(communicator(),
	                    configLoader.get<std::string>("Proxies.Lidar3D"), "Lidar3DProxy", lidar3d_proxy);
	require<RoboCompOmniRobot::OmniRobotPrx, RoboCompOmniRobot::OmniRobotPrxPtr>(communicator(),
	                    configLoader.get<std::string>("Proxies.OmniRobot"), "OmniRobotProxy", omnirobot_proxy);

	//Topic Manager code

	IceStorm::TopicManagerPrxPtr topicManager;
	try
	{
		topicManager = Ice::checkedCast<IceStorm::TopicManagerPrx>(communicator()->stringToProxy(configLoader.get<std::string>("Proxies.TopicManager")));
		if (!topicManager)
		{
		    std::cout << "[" << PROGRAM_NAME << "]: TopicManager.Proxy not defined in config file."<<std::endl;
		    std::cout << "	 Config line example: TopicManager.Proxy=IceStorm/TopicManager:default -p 9999"<<std::endl;
	        return EXIT_FAILURE;
		}
	}
	catch (const Ice::Exception &ex)
	{
		std::cout << "[" << PROGRAM_NAME << "]: Exception: 'rcnode' not running: " << ex << std::endl;
		return EXIT_FAILURE;
	}

	tprx = std::make_tuple(differentialrobot_proxy,lidar3d_proxy,omnirobot_proxy);
	SpecificWorker *worker = new SpecificWorker(this->configLoader, tprx, startup_check_flag);
	QObject::connect(worker, SIGNAL(kill()), &a, SLOT(quit()));

	try
	{

		//Subscribe code
		subscribe<FullPoseEstimationPubI>(communicator(),
		                    topicManager, configLoader.get<std::string>("Endpoints.FullPoseEstimationPubTopic"),
						    configLoader.get<std::string>("Endpoints.FullPoseEstimationPubPrefix"), "FullPoseEstimationPub", worker,  0,
						    fullposeestimationpub_topic, fullposeestimationpub, PROGRAM_NAME);

		// Server adapter creation and publication
		std::cout << SERVER_FULL_NAME " started" << std::endl;

		// User defined QtGui elements ( main window, dialogs, etc )

		#ifdef USE_QTGUI
			//ignoreInterrupt(); // Uncomment if you want the component to ignore console SIGINT signal (ctrl+c).
			a.setQuitOnLastWindowClosed( true );
		#endif
		// Run QT Application Event Loop
		a.exec();

		try
		{
			std::cout << "Unsubscribing topic: fullposeestimationpub " <<std::endl;
			fullposeestimationpub_topic->unsubscribe(fullposeestimationpub);

		}
		catch(const Ice::Exception& ex)
		{
			std::cout << "ERROR Unsubscribing" << ex.what()<<std::endl;
		}


		status = EXIT_SUCCESS;
	}
	catch(const Ice::Exception& ex)
	{
		status = EXIT_FAILURE;

		std::cerr << "[" << PROGRAM_NAME << "]: Exception raised on main thread: " << std::endl;
		std::cerr << ex;

	}
	#ifdef USE_QTGUI
		a.quit();
	#endif

	status = EXIT_SUCCESS;
	delete worker;
	return status;
}

int main(int argc, char* argv[])
{
	std::string arg;

	// Set config file
	QString configFile("etc/config");
	bool startup_check_flag = false;
	QString prefix("");
	if (argc > 1)
	{

		// Search in argument list for arguments
		QString startup = QString("--startup-check");
		QString initIC = QString("--Ice.Config=");
		QString prfx = QString("--prefix=");
		for (int i = 0; i < argc; ++i)
		{
			arg = argv[i];
			if (arg.find(startup.toStdString(), 0) != std::string::npos)
			{
				startup_check_flag = true;
				std::cout << "Startup check = True"<< std::endl;
			}
			else if (arg.find(prfx.toStdString(), 0) != std::string::npos)
			{
				prefix = QString::fromStdString(arg).remove(0, prfx.size());
				if (prefix.size()>0)
					prefix += QString(".");
				printf("Configuration prefix: <%s>\n", prefix.toStdString().c_str());
			}
			else if (arg.find(initIC.toStdString(), 0) != std::string::npos)
			{
				configFile = QString::fromStdString(arg).remove(0, initIC.size());
				qDebug()<<__LINE__<<"Starting with config file:"<<configFile;
			}
			else if (i==1 and argc==2 and arg.find("--", 0) == std::string::npos)
			{
				configFile = QString::fromStdString(arg);
				qDebug()<<__LINE__<<QString::fromStdString(arg)<<argc<<arg.find("--", 0)<<"Starting with config file:"<<configFile;
			}
		}

	}
	mapping_component app(configFile, prefix, startup_check_flag);

	return app.main(argc, argv, app.getInitializationDataIce());
}
