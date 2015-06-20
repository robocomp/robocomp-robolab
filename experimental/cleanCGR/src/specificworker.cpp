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
 */
#include "specificworker.h"

#include "vectorparticlefilter.h"
#include "vector_map.h"
//#include "popt_pp.h"
#include "terminal_utils.h"
#include "timer.h"
#include "proghelp.h"
#include "configreader.h"
//#include "plane_filtering.h"

using namespace std;

bool run = true;
bool usePointCloud = false;
bool noLidar = false;
int numParticles = 20;
int debugLevel = -1;

vector2f initialLoc;
float initialAngle;
float locUncertainty, angleUncertainty;

VectorLocalization2D *localization;

VectorLocalization2D::PointCloudParams pointCloudParams;
VectorLocalization2D::LidarParams lidarParams;
VectorLocalization2D::MotionModelParams motionParams;

vector<vector2f> pointCloud;
vector<vector2f> pointCloudNormals;

string curMapName;
vector2f curLoc;
float curAngle;
double curTime;



/**
* 	Load config params
*/
inline void LoadParameters()
{
  WatchFiles watch_files;
  ConfigReader config("../_etc/"); //path a los ficheros de configuracion desde el path del binario.
  config.init(watch_files);
  
  config.addFile("config/localization_parameters.cfg");
  
  if(!config.readFiles()){
    printf("Failed to read config\n");
    exit(1);
  }
 
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







/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	LoadParameters();
	printf("bien cargados");
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{



	
	timer.start(Period);

	return true;
}

void SpecificWorker::compute()
{
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


void SpecificWorker::newFilteredPoints(const OrientedPoints &ops)
{

}






