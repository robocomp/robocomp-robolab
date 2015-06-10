/*
 *    Copyright (C) 2015 by YOUR NAME HERE
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
//========================================================================
/*!
* \based in Joydeep Biswas, (C) 2010 paper
*/
//========================================================================
 */


#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <string.h>

// #include <ros/ros.h>
// #include <tf/tf.h>
// #include <tf/transform_listener.h>
// #include <tf/transform_broadcaster.h>
// #include <sensor_msgs/LaserScan.h>
// #include <sensor_msgs/Image.h>
// #include <nav_msgs/Odometry.h>
// #include <geometry_msgs/PoseArray.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
// #include <ros/package.h>

#include "vectorparticlefilter.h"
#include "vector_map.h"
#include "popt_pp.h"
#include "terminal_utils.h"
#include "timer.h"
#include "proghelp.h"
#include "cgr_localization/DisplayMsg.h"
#include "cgr_localization/LocalizationInterfaceSrv.h"
#include "cgr_localization/LocalizationMsg.h"
#include "configreader.h"
#include "plane_filtering.h"

#include "specificworker.h"

using namespace std;
/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
    LoadParameters();
    //========================== Set up Command line parameters =======================
  static struct poptOption options[] = {
    { "num-particles",    'n', POPT_ARG_INT,     &numParticles,       0, "Number of Particles",   "NUM"},
    { "debug",            'd', POPT_ARG_INT,     &debugLevel,         0, "Debug Level",           "NUM"},
    { "start-x",          'x', POPT_ARG_DOUBLE,  &initialLoc.x,       0, "Starting X location",   "NUM"},
    { "start-y",          'y', POPT_ARG_DOUBLE,  &initialLoc.y,       0, "Starting Y location",   "NUM"},
    { "start-angle",      'a', POPT_ARG_DOUBLE,  &initialAngle,       0, "Starting Angle",        "NUM"},
    { "use-point-cloud",  'p', POPT_ARG_NONE,    &usePointCloud,      0, "Use Point Cloud",       "NONE"},
    { "no-lidar",         'l', POPT_ARG_NONE,    &noLidar,            0, "No LIDAR Observations", "NONE"},
    { "Alpha1-param",     'u', POPT_ARG_DOUBLE,  &motionParams.Alpha1,  0, "Alpha1 parameter",   "NUM"},
    { "Alpha2-param",     'v', POPT_ARG_DOUBLE,  &motionParams.Alpha2,  0, "Alpha2 parameter",   "NUM"},
    { "Alpha3-param",     'w', POPT_ARG_DOUBLE,  &motionParams.Alpha3,  0, "Alpha3 parameter",   "NUM"},
   
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };
  
   // parse options
  POpt popt(NULL,argc,(const char**)argv,options,0);
  int c;
  while((c = popt.getNextOpt()) >= 0){
  }
 
  //========================= Welcome screen, Load map & log ========================
  ColourTerminal(TerminalUtils::TERMINAL_COL_WHITE,TerminalUtils::TERMINAL_COL_BLACK,TerminalUtils::TERMINAL_ATTR_BRIGHT);
  printf("\nVector Localization\n\n");
  ResetTerminal();
 
  if(debugLevel>=0){
    printf("NumParticles     : %d\n",numParticles);
    printf("Alpha1           : %f\n",motionParams.Alpha1);
    printf("Alpha2           : %f\n",motionParams.Alpha2);
    printf("Alpha3           : %f\n",motionParams.Alpha3);
    printf("UsePointCloud    : %d\n",usePointCloud?1:0);
    printf("UseLIDAR         : %d\n",noLidar?0:1);
    printf("Visualizations   : %d\n",debugLevel>=0?1:0);
    printf("\n");
  }
  double seed = floor(fmod(GetTimeSec()*1000000.0,1000000.0));
  if(debugLevel>-1) printf("Seeding with %d\n",(unsigned int)seed);
  srand(seed);
 
  //Initialize particle filter, sensor model, motion model, refine model
  string mapsFolder(ros::package::getPath("cgr_localization").append("/maps/"));
  localization = new VectorLocalization2D(mapsFolder.c_str());
  InitModels();
 
  //Initialize particle filter
  localization->initialize(numParticles,curMapName.c_str(),initialLoc,initialAngle,locUncertainty,angleUncertainty);
 
  //Initialize ros for sensor and odometry topics
  InitHandleStop(&run);
  ros::init(argc, argv, "CGR_Localization");
  ros::NodeHandle n;
  guiPublisher = n.advertise<DisplayMsg>("localization_gui",1,true);
  localizationPublisher = n.advertise<LocalizationMsg>("localization",1,true);
  particlesPublisher = n.advertise<geometry_msgs::PoseArray>("particlecloud",1,false);
  Sleep(0.1);
 
  localizationServer = n.advertiseService("localization_interface", &localizationCallback);
 
  //Initialize ros for sensor and odometry topics
  ros::Subscriber odometrySubscriber = n.subscribe("odom", 20, odometryCallback);
  ros::Subscriber lidarSubscriber = n.subscribe("scan", 5, lidarCallback);
  ros::Subscriber kinectSubscriber = n.subscribe("kinect_depth", 1, depthCallback);
  ros::Subscriber initialPoseSubscriber = n.subscribe("initialpose",1,initialPoseCallback);
  transformListener = new tf::TransformListener(ros::Duration(10.0));
  transformBroadcaster = new tf::TransformBroadcaster(); 
 
  filteredPointCloudPublisher = n.advertise<sensor_msgs::PointCloud>("Cobot/Kinect/FilteredPointCloud", 1);
 
  while(ros::ok() && run){
    ros::spinOnce();
    publishGUI();
    publishLocation();
    Sleep(0.005);
  }
 
  if(debugLevel>=0) printf("closing.\n");
  return 0;
}

  
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

void SpecificWorker::LoadParameters()
{
  WatchFiles watch_files;
  ConfigReader config(ros::package::getPath("cgr_localization").append("/").c_str());
  
  config.init(watch_files);
    
  {
    ConfigReader::SubTree c(config,"initialConditions");
    
    bool error = false;
    curMapName = string(c.getStr("mapName"));
    error = error || curMapName.length()==0;
    error = error || !c.getVec2f("loc",initialLoc);
    error = error || !c.getReal("angle", initialAngle);
    error = error || !c.getReal("locUncertainty", locUncertainty);
    error = error || !c.getReal("angleUncertainty", angleUncertainty);
    
    if(error){
      printf("Error Loading Initial Conditions!\n");
      exit(2);
    }
  }
  
  {
    ConfigReader::SubTree c(config,"motionParams");
    
    bool error = false;
    error = error || !c.getReal("Alpha1", motionParams.Alpha1);
    error = error || !c.getReal("Alpha2", motionParams.Alpha2);
    error = error || !c.getReal("Alpha3", motionParams.Alpha3);
    error = error || !c.getReal("kernelSize", motionParams.kernelSize);
    
    
    if(error){
      printf("Error Loading Predict Parameters!\n");
      exit(2);
    }
  }
  
  {
    ConfigReader::SubTree c(config,"lidarParams");
    
    bool error = false;
    // Laser sensor properties
    error = error || !c.getReal("angleResolution", lidarParams.angleResolution);
    error = error || !c.getInt("numRays", lidarParams.numRays);
    error = error || !c.getReal("maxRange", lidarParams.maxRange);
    error = error || !c.getReal("minRange", lidarParams.minRange);
    
    // Pose of laser sensor on robot
    vector2f laserToBaseTrans;
    float xRot, yRot, zRot;
    error = error || !c.getVec2f<vector2f>("laserToBaseTrans", laserToBaseTrans);
    error = error || !c.getReal("xRot", xRot);
    error = error || !c.getReal("yRot", yRot);
    error = error || !c.getReal("zRot", zRot);
    Matrix3f laserToBaseRot;
    laserToBaseRot = AngleAxisf(xRot, Vector3f::UnitX()) * AngleAxisf(yRot, Vector3f::UnitY()) * AngleAxisf(zRot, Vector3f::UnitZ());
    lidarParams.laserToBaseTrans = Vector2f(V2COMP(laserToBaseTrans));
    lidarParams.laserToBaseRot = laserToBaseRot.block(0,0,2,2);
    
    // Parameters related to observation update
    error = error || !c.getReal("logObstacleProb", lidarParams.logObstacleProb);
    error = error || !c.getReal("logOutOfRangeProb", lidarParams.logOutOfRangeProb);
    error = error || !c.getReal("logShortHitProb", lidarParams.logShortHitProb);
    error = error || !c.getReal("correlationFactor", lidarParams.correlationFactor);
    error = error || !c.getReal("lidarStdDev", lidarParams.lidarStdDev);
    error = error || !c.getReal("attractorRange", lidarParams.attractorRange);
    error = error || !c.getReal("kernelSize", lidarParams.kernelSize);
    
    // Parameters related to observation refine
    error = error || !c.getInt("minPoints", lidarParams.minPoints);
    error = error || !c.getInt("numSteps", lidarParams.numSteps);
    error = error || !c.getReal("etaAngle", lidarParams.etaAngle);
    error = error || !c.getReal("etaLoc", lidarParams.etaLoc);
    error = error || !c.getReal("maxAngleGradient", lidarParams.maxAngleGradient);
    error = error || !c.getReal("maxLocGradient", lidarParams.maxLocGradient);
    error = error || !c.getReal("minCosAngleError", lidarParams.minCosAngleError);
    error = error || !c.getReal("correspondenceMargin", lidarParams.correspondenceMargin);
    error = error || !c.getReal("minRefineFraction", lidarParams.minRefineFraction);
    
    lidarParams.initialize();
    
    if(error){
      printf("Error Loading Lidar Parameters!\n");
      exit(2);
    }
  }
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//       THE FOLLOWING IS JUST AN EXAMPLE
//
// 	try
// 	{
// 		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
// 		innermodel_path=par.value;
// 		innermodel = new InnerModel(innermodel_path);
// 	}
// 	catch(std::exception e) { qFatal("Error reading config params"); }
	
	
	timer.start(Period);

	return true;
}


void SpecificWorker::compute()
{
static RoboCompLaser::TLaserData laserData;

 		//rgbd_proxy->getXYZ(points, hState, bState);
		laserData = laser_proxy->getLaserData();
			int j=0;
			for(auto i : laserData)
		{
			//pasar a xyz añadiendo w QVec p1 = QVec::vec3(x,y,z); 
			//points[j].x=0;
			//points[j].y=0;
			//points[j].z=0;
			//points[j].w=0;
			//j++;
		}
// 	try
// 	{
// 		camera_proxy->getYImage(0,img, cState, bState);
// 		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
// 		searchTags(image_gray);
// 	}
// 	catch(const Ice::Exception &e)
// 	{
// 		std::cout << "Error reading from Camera" << e << std::endl;
// 	}
}






