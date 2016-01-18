/*
 *    Copyright (C) 2006-2011 by RoboLab - University of Extremadura
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
/** \mainpage RoboComp::laserrgbComp
 *
 * \section intro_sec Introduction
 *
 * The laserrgbComp component...
 *
 * \section interface_sec Interface
 *
 * laserrgbComp interface...
 *
 * \section install_sec Installation
 *
 * \subsection install1_ssec Software depencences
 * laserrgbComp ...
 *
 * \subsection install2_ssec Compile and install
 * cd laserrgbComp
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
 * The configuration file laserrgbComp/etc/config ...
 * </p>
 *
 * \subsection execution_ssec Execution
 *
 * Just: "${PATH_TO_BINARY}/laserrgbComp --Ice.Config=${PATH_TO_CONFIG_FILE}"
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
#include "monitor.h"
#include "worker.h"
#include "commonbehaviorI.h"
#include <laserI.h>


#include <Laser.h>
#include <ui_guiDlg.h>
#include <OmniRobot.h>
#include <JointMotor.h>
#include <RGBD.h>
#include <RGBDBus.h>


// User includes here

// Namespaces
using namespace std;
using namespace RoboCompCommonBehavior;
using namespace RoboCompLaser;

using namespace RoboCompOmniRobot;
using namespace RoboCompLaser;
using namespace RoboCompJointMotor;
using namespace RoboCompRGBD;
using namespace RoboCompRGBDBus;


class laserRGBComp : public RoboComp::Application
{
private:
	// User private data here

	void initialize();
	void decimateProtoPointClouds(PointCloudMap &map, RoboCompRGBDBus::CameraParamsMap &cameras, int decimation);
public:
	virtual int run(int, char*[]);
};

void laserRGBComp::initialize()
{

}

void laserRGBComp::decimateProtoPointClouds(PointCloudMap &mapO, RoboCompRGBDBus::CameraParamsMap &cameras, int decimation)
{
	printf("Decimating %d\n", decimation);
	PointCloudMap map2 = mapO;

	
	RoboCompRGBDBus::PointCloudMap::iterator iter;
	for (iter=map2.begin(); iter!=map2.end(); iter++)
	{
		const int32_t width  = cameras[iter->first].depthWidth;
		const int32_t height = cameras[iter->first].depthHeight;
		printf("Decimating (%d)  %dx%d  [%s]\n", decimation, width, height, iter->first.c_str());
		
		RoboCompRGBDBus::PointCloud pcN = mapO[iter->first];
		printf("O: %ld\n", pcN.size());
		RoboCompRGBDBus::PointCloud pcD;
		int pass = pow(2, decimation-1);
		for (int hi=0; hi<height; hi+=pass)
		{
			for (int wi=0; wi<width; wi+=pass)
			{
				pcD.push_back(pcN[hi*width+wi]);
			}
		}
		map2[iter->first] = pcD;
		printf("R: %ld\n", pcD.size());
	}

	mapO = map2;

}


int laserRGBComp::run(int argc, char* argv[])
{
#ifdef USE_QTGUI
	QApplication a(argc, argv);  // GUI application
#else
	QCoreApplication a(argc, argv);  // NON-GUI application
#endif
	int status=EXIT_SUCCESS;
	string proxy;
	
	WorkerConfig cfg;
	

	configGetString("InnerModelPath", cfg.xmlpath, "");
	printf("InnerModelPath: %s\n", cfg.xmlpath.c_str());
	
	configGetString("LaserBaseID", cfg.laserBaseID, "");
	printf("LaserBaseID: %s\n", cfg.laserBaseID.c_str());
	
	configGetFloat("MinHeight", cfg.minHeight, 0);
	printf("MinHeight: %f\n", cfg.minHeight);
	
	configGetFloat("MaxHeight", cfg.maxHeight, 0);
	printf("MaxHeight: %f\n", cfg.maxHeight);
	
	configGetFloat("MinHeightNeg", cfg.minHeightNeg, 0);
	printf("MinHeightNeg: %f\n", cfg.minHeightNeg);
	
	configGetInt("LaserSize", cfg.LASER_SIZE, 0);
	printf("LaserSize: %d\n", cfg.LASER_SIZE);
	
	configGetFloat("MinRange", cfg.MIN_LENGTH, 0);
	printf("MinRange: %f\n", cfg.MIN_LENGTH);
	
	configGetFloat("MaxRange", cfg.maxLength, 0);
	printf("MaxRange: %f\n", cfg.maxLength);
	
	configGetFloat("FOV", cfg.FOV, 0);
	printf("FOV: %f\n", cfg.FOV);
	
	configGetInt("DecimationLevel", cfg.DECIMATION_LEVEL, 0);
	printf("DecimationLevel: %d\n", cfg.DECIMATION_LEVEL);
	
	configGetString("ActualLaserID", cfg.actualLaserID, "");
	printf("ActualLaserID: %s\n", cfg.actualLaserID.c_str());
	
	std::string ssss;
	configGetString("UpdateJoint", ssss, "y");
	if (ssss[0] == 't' or ssss[0] == 'T' or ssss[0] == '1' or ssss[0] == 'Y' or ssss[0] == 'y')
		cfg.updateJoint = true;
	else if (ssss[0] == 'f' or ssss[0] == 'F' or ssss[0] == '0' or ssss[0] == 'N' or ssss[0] == 'n')
		cfg.updateJoint = false;
	else
		qFatal("Wrong UpdateJoint value");
	printf("UpdateJoint: %d\n", cfg.updateJoint);
	


	
	int numRGBD;
	configGetInt("RGBDNumber", numRGBD);
	printf("RGBDNumber: %d\n", numRGBD);
	std::vector<LaserRGBDInfo> rgbds;
	for (int i=1; i<=numRGBD; ++i)
	{
		LaserRGBDInfo lInfo;

		std::string prxStr = "RGBDProxy"+QString::number(i).toStdString();
		printf("*** Proxy string: %s\n", prxStr.c_str());
		proxy = getProxyString(prxStr);
		// Load the remote server proxy (RGBD)
		bool ok = false;
		try
		{
			lInfo.proxyRGBD = RGBDPrx::uncheckedCast( communicator()->stringToProxy( proxy ) );
			RoboCompRGBD::PointSeq points;
			RoboCompJointMotor::MotorStateMap hState;
			RoboCompDifferentialRobot::TBaseState bState;
			lInfo.proxyRGBD->getXYZ(points, hState, bState);

			if( !lInfo.proxyRGBD )
			{
				rInfo(QString("Error loading proxy as RGBD, let's try with RGBDBus!"));
			}
			else
			{
				rInfo("RGBD proxy initialized Ok!");
				try
				{
					std::string idStr = std::string("RGBDID")+QString::number(i).toStdString();
					configGetString(idStr, lInfo.id);
					printf("%s\n", lInfo.id.c_str());
					printf("idStr: %s\n", idStr.c_str());
					printf("%s\n", lInfo.id.c_str());
					ok = true;
				}
				catch(...)
				{
					qFatal("Can't read ID string for proxy number: %d\n", i);
				}
				lInfo.bus = false;
				printf("\n\nfalse\n\n");
				rgbds.push_back(lInfo);
			}
		}
		catch(const Ice::Exception& ex)
		{
			rInfo(QString("Error loading proxy as RGBD, let's try with RGBDBus!"));
		}
		if (not ok)
		{
			try
			{
				lInfo.proxyRGBDBus = RGBDBusPrx::uncheckedCast( communicator()->stringToProxy( proxy ) );
				if (!lInfo.proxyRGBDBus)
				{
					rInfo(QString("Error loading proxy as RGBD or RGBDBus!"));
					return EXIT_FAILURE;
				}
				try
				{
							std::string idStr = std::string("RGBDID")+QString::number(i).toStdString();
							configGetString(idStr, lInfo.id);
							printf("%s\n", lInfo.id.c_str());
							printf("idStr: %s\n", idStr.c_str());
							printf("%s\n", lInfo.id.c_str());
							ok = true;
				}
				catch(...)
				{
							qFatal("Can't read ID string for proxy number: %d\n", i);
				}

				rInfo("RGBDBus proxy initialized Ok!");
				/// Get parameters of the cameras in the bus
				lInfo.cameras = (lInfo.proxyRGBDBus)->getAllCameraParams();
				/// All camera list
				pair<string, RoboCompRGBDBus::CameraParams> me;
				vector<string> v;
				for (RoboCompRGBDBus::CameraParamsMap::iterator iter=lInfo.cameras.begin(); iter!=lInfo.cameras.end(); iter++)
				{
					v.push_back(iter->first);
				}
				/// Get and decimate protoPointClouds
				(lInfo.proxyRGBDBus)->getProtoClouds(v, lInfo.protoPointClouds);
				if (cfg.DECIMATION_LEVEL > 0)
					decimateProtoPointClouds(lInfo.protoPointClouds, lInfo.cameras, cfg.DECIMATION_LEVEL);
				/// Set bus
				lInfo.bus = true;
				printf("\n\ntrue\n\n");
				rgbds.push_back(lInfo);
			}
			catch(const Ice::Exception& ex)
			{
				cout << "rgbdbus [" << PROGRAM_NAME << "]: Exception: " << ex << endl;
				return EXIT_FAILURE;
			}
		}
	}
	cfg.rgbdsVec = rgbds;

	printf("Total RGBDs: %d\n", (int)rgbds.size());
	for (int i = 0; i < (int)rgbds.size(); ++i)
	{
		printf("-> %d\n", rgbds[i].bus);
	}

	LaserPrx laser_proxy;
	OmniRobotPrx omnirobot_proxy;
	JointMotorPrx jointmotor_proxy;


	try
	{
		// Load the remote server proxy
		proxy = getProxyString("OmniRobotProxy");
		omnirobot_proxy = OmniRobotPrx::uncheckedCast( communicator()->stringToProxy( proxy ) );
		if( !omnirobot_proxy )
		{
			rInfo(QString("Error loading proxy!"));
			return EXIT_FAILURE;
		}
	}
	catch(const Ice::Exception& ex)
	{
		cout << "omni [" << PROGRAM_NAME << "]: Exception: " << ex << endl;
		return EXIT_FAILURE;
	}
	rInfo("OmniRobotProxy initialized Ok!");

	
	
	if (cfg.actualLaserID.size() > 0)
	{
		try
		{
			// Load the remote server proxy
			proxy = getProxyString("LaserProxy");
			laser_proxy = LaserPrx::uncheckedCast( communicator()->stringToProxy( proxy ) );
			if( !laser_proxy )
			{
				rInfo(QString("Error loading proxy!"));
				return EXIT_FAILURE;
			}
		}
		catch(const Ice::Exception& ex)
		{
			cout << "laser [" << PROGRAM_NAME << "]: Exception: " << ex << endl;
			return EXIT_FAILURE;
		}
		rInfo("LaserProxy initialized Ok!");
	}
	else
	{
		printf("Not using any real laser\n");
	}

	if (cfg.updateJoint)
	{
		try
		{
			proxy = getProxyString("JointMotorProxy");
			printf("joint string <%s>\n%d\n", proxy.c_str(), (int)proxy.size());
			jointmotor_proxy = JointMotorPrx::uncheckedCast( communicator()->stringToProxy( proxy ) );
			if (proxy.size() > 4)
			{
				if( !jointmotor_proxy )
				{
					qFatal("Error loading joint proxy! %s:%d", __FILE__,  __LINE__);
				}
			}
			else
			{
				qFatal("Error loading joint proxy! %s:%d", __FILE__,  __LINE__);
			}
		}
		catch(const Ice::Exception& ex)
		{
			cout << "joint [" << PROGRAM_NAME << "]: Exception: " << ex << endl;
			qFatal("Error loading joint proxy! %s:%d", __FILE__,  __LINE__);
		}
		rInfo("JointMotorProxy initialized Ok!");
	}




	Worker *worker = new Worker(omnirobot_proxy, jointmotor_proxy, laser_proxy, cfg);
	//Monitor thread
	Monitor *monitor = new Monitor(worker,communicator());
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
		Ice::ObjectAdapterPtr adapter = communicator()->createObjectAdapter("laserRGBComp");
		LaserI *laserI = new LaserI(worker );
		adapter->add(laserI, communicator()->stringToIdentity("laser"));

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
	laserRGBComp app;

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
