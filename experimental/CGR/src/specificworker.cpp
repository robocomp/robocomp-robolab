//
//    Copyright (C) 2015 by YOUR NAME HERE
//
//    This file is part of RoboComp
//
//    RoboComp is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    RoboComp is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
//
// based in Joydeep Biswas, (C) 2010 paper
//
//========================================================================
//


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

//#include "popt_pp.h"
#include "proghelp.h"
//#include "cgr_localization/DisplayMsg.h"
//#include "cgr_localization/LocalizationInterfaceSrv.h"
//#include "cgr_localization/LocalizationMsg.h"

#include "vectorparticlefilter.h"
#include "vector_map.h"

#include "terminal_utils.h"
#include "timer.h"



#include "configreader.h"
//#include "plane_filtering.h"

#include "specificworker.h"

bool run = true;
bool usePointCloud = false;
bool noLidar = false;
int numParticles = 20;
int debugLevel = -1;

vector2f initialLoc;
float initialAngle;
float locUncertainty, angleUncertainty;

VectorLocalization2D *localization;

using namespace std;
/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

    LoadParameters();
/* 
    printf("NumParticles     : %d\n",numParticles);
    printf("Alpha1           : %f\n",motionParams.Alpha1);
    printf("Alpha2           : %f\n",motionParams.Alpha2);
    printf("Alpha3           : %f\n",motionParams.Alpha3);
    printf("UsePointCloud    : %d\n",usePointCloud?1:0);
    printf("UseLIDAR         : %d\n",noLidar?0:1);
    printf("Visualizations   : %d\n",debugLevel>=0?1:0);
    printf("\n");
*/  
  double seed = floor(fmod(GetTimeSec()*1000000.0,1000000.0));
  if(debugLevel>-1) printf("Seeding with %d\n",(unsigned int)seed);
  srand(seed);

 


//Initialize particle filter, sensor model, motion model, refine model
  string mapsFolder("../maps");
//  localization = new VectorLocalization2D(mapsFolder.c_str());
  localization = new VectorLocalization2D(mapsFolder.c_str());
  localization->initialize(numParticles,
	curMapName.c_str(),initialLoc,initialAngle,locUncertainty,angleUncertainty);
  	
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
  ConfigReader config("../etc/"); //path a los ficheros de configuracion desde el path del binario.
  config.init(watch_files);
  
  config.addFile("localization_parameters.cfg");
  config.addFile("kinect_parameters.cfg");
  if(!config.readFiles()){
    printf("Failed to read config\n");
    exit(1);
  }
/**
    {
    ConfigReader::SubTree c(config,"KinectParameters");
    
    unsigned int maxDepthVal;
    bool error = false;
    error = error || !c.getReal("f",kinectDepthCam.f);
    error = error || !c.getReal("fovH",kinectDepthCam.fovH);
    error = error || !c.getReal("fovV",kinectDepthCam.fovV);
    error = error || !c.getInt("width",kinectDepthCam.width);
    error = error || !c.getInt("height",kinectDepthCam.height);
    error = error || !c.getUInt("maxDepthVal",maxDepthVal);
    kinectDepthCam.maxDepthVal = maxDepthVal;    
    
    vector3f kinectLoc;
    float xRot, yRot, zRot;
    error = error || !c.getVec3f("loc",kinectLoc);
    error = error || !c.getReal("xRot",xRot);
    error = error || !c.getReal("yRot",yRot);
    error = error || !c.getReal("zRot",zRot);
    kinectToRobotTransform.xyzRotationAndTransformation(xRot,yRot,zRot,kinectLoc);
    
    if(error){
      printf("Error Loading Kinect Parameters!\n");
      exit(2);
    }
    }

    {
    ConfigReader::SubTree c(config,"PlaneFilteringParameters");
    
    bool error = false;
    error = error || !c.getUInt("maxPoints",filterParams.maxPoints);
    error = error || !c.getUInt("numSamples",filterParams.numSamples);
    error = error || !c.getUInt("numLocalSamples",filterParams.numLocalSamples);
    error = error || !c.getUInt("planeSize",filterParams.planeSize);
    error = error || !c.getReal("maxError",filterParams.maxError);
    error = error || !c.getReal("maxDepthDiff",filterParams.maxDepthDiff);
    error = error || !c.getReal("minInlierFraction",filterParams.minInlierFraction);
    error = error || !c.getReal("WorldPlaneSize",filterParams.WorldPlaneSize);
    error = error || !c.getUInt("numRetries",filterParams.numRetries);
    filterParams.runPolygonization = false;
    
    if(error){
      printf("Error Loading Plane Filtering Parameters!\n");
      exit(2);
    }
  }
*/ 
  
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
  /*
  {
    ConfigReader::SubTree c(config,"lidarParams");
    
    bool error = false;
    // Laser sensor properties  //BASURA
    error = error || !c.getReal("angleResolution", lidarParams.angleResolution);
    error = error || !c.getInt("numRays", lidarParams.numRays);
    error = error || !c.getReal("maxRange", lidarParams.maxRange);
    error = error || !c.getReal("minRange", lidarParams.minRange);
    
    // Pose of laser sensor on robot	//BASURA
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
*/
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

	// Obtener la posición del robot en el mundo. Al arrancar 0
	// LLamar a omnirobot -> getBaseState();

	// Si hay algo nuevo
	// Si no
		// llamar a localization -> predict

}

void SpecificWorker::filterParticle()
{
  //Call particle filter
    vector<vector2f> pointCloud2D, pointCloudNormals2D;

    for(unsigned int i=0; i<filteredPointCloud.size(); i++){
      if(fabs(pointCloudNormals[i].z)>sin(RAD(30.0)))
        continue;

      vector2f curNormal(V2COMP(pointCloudNormals[i]));
      vector2f curPoint(V2COMP(filteredPointCloud[i]));
      curNormal.normalize();
      pointCloudNormals2D.push_back(curNormal);
      pointCloud2D.push_back(curPoint);
    }

    double start = GetTimeSec();
    localization->refinePointCloud(pointCloud2D, pointCloudNormals2D, pointCloudParams);
    localization->updatePointCloud(pointCloud2D, pointCloudNormals2D, motionParams, pointCloudParams);
    localization->resample(VectorLocalization2D::SparseMultinomialResampling);
    localization->computeLocation(curLoc,curAngle);
    std::cerr << "point cloud update: " << GetTimeSec()-start << std::endl;
  }

////////////////////////////
///  SERVANTS
////////////////////////////


void SpecificWorker::newFilteredPoints(const OrientedPoints &ops)
{
//	qDebug() << "hola";
//	for (int i = 0; i < ops.size(); i++)
//	{
//		cout << "Points: "<<ops[i].x<<" "<< ops[i].y<<" "<< ops[i].z<<endl;
//		cout << "Normals: "<<ops[i].nx<<" "<< ops[i].ny<<" "<< ops[i].nz<<endl;
//	}
}



//static RoboCompLaser::TLaserData laserData;

//	laserData = laser_proxy->getLaserData();
//	int j=0;
//	for(auto i : laserData)
//	{
		//pasar a xyz añadiendo w QVec p1 = QVec::vec3(x,y,z); 
		//points[j].x=0;
		//points[j].y=0;
		//points[j].z=0;
		//points[j].w=0;
		//j++;
//	}


