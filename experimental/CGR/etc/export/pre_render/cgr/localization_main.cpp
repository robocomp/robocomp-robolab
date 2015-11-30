//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
* \file    localization_main.cpp
* \brief   Main Vector Localization program
* \author  Joydeep Biswas, (C) 2010
*/
//========================================================================

#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <string.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/package.h>

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

using namespace ros;
using namespace cgr_localization;
Publisher guiPublisher;
Publisher localizationPublisher;
Publisher filteredPointCloudPublisher;
ServiceServer localizationServer;
Publisher particlesPublisher;
tf::TransformBroadcaster *transformBroadcaster;
tf::TransformListener *transformListener;
DisplayMsg guiMsg;
sensor_msgs::PointCloud filteredPointCloudMsg; /// FSPF point cloud

VectorLocalization2D::PointCloudParams pointCloudParams;
VectorLocalization2D::LidarParams lidarParams;
VectorLocalization2D::MotionModelParams motionParams;

vector<vector2f> pointCloud;
vector<vector2f> pointCloudNormals;

string curMapName;
vector2f curLoc;
float curAngle;
double curTime;
sensor_msgs::LaserScan lastLidarMsg;
sensor_msgs::Image lastDepthMsg;
geometry_msgs::PoseArray particlesMsg;

//Point Cloud parameters
GVector::matrix3d<float> kinectToRobotTransform;
KinectRawDepthCam kinectDepthCam;
PlaneFilter::PlaneFilterParams filterParams;
PlaneFilter planeFilter;

void publishGUI();
void lidarCallback(const sensor_msgs::LaserScan& msg);
void depthCallback(const sensor_msgs::Image& msg);
void publishLocation(bool limitRate=true);

bool localizationCallback(LocalizationInterfaceSrv::Request& req, LocalizationInterfaceSrv::Response& res)
{  
  vector2f loc(req.loc_x, req.loc_y);
  if(debugLevel>0) printf("Setting location: %f %f %f\u00b0 on %s\n",V2COMP(loc),DEG(req.orientation),req.map.c_str());
  localization->setLocation(loc, req.orientation,req.map.c_str(),0.01,RAD(1.0));
  return true;
}

void ClearGUI()
{
  guiMsg.lines_p1x.clear();
  guiMsg.lines_p1y.clear();
  guiMsg.lines_p2x.clear();
  guiMsg.lines_p2y.clear();
  guiMsg.points_x.clear();
  guiMsg.points_y.clear();
  guiMsg.lines_col.clear();
  guiMsg.points_col.clear();
  guiMsg.circles_x.clear();
  guiMsg.circles_y.clear();
  guiMsg.circles_col.clear();
  
  guiMsg.windowSize = 1.0;
}

void drawPointCloud()
{
  //printf("publishing %d points\n",(int) pointCloud.size());
  
  for(int i=0; i<(int) pointCloud.size(); i++){
    guiMsg.points_x.push_back(pointCloud[i].x);
    guiMsg.points_y.push_back(pointCloud[i].y);
    guiMsg.points_col.push_back(0xDE2352);
  }
}

void publishLocation(bool limitRate)
{
  static double tLast = 0;
  if(GetTimeSec()-tLast<0.03 && limitRate)
    return;
  tLast = GetTimeSec();
  LocalizationMsg msg;
  localization->computeLocation(curLoc, curAngle);
  msg.timeStamp = GetTimeSec();
  msg.x = curLoc.x;
  msg.y = curLoc.y;
  msg.angle = curAngle;
  msg.map = string(localization->getCurrentMapName());
  
  localization->getUncertainty(msg.angleUncertainty, msg.locationUncertainty);
  
  VectorLocalization2D::EvalValues laserEval, pointCloudEval;
  localization->getEvalValues(laserEval,pointCloudEval);
  msg.laserNumCorrespondences = laserEval.numCorrespondences;
  msg.laserNumObservedPoints = laserEval.numObservedPoints;
  msg.laserStage0Weights = laserEval.stage0Weights;
  msg.laserStageRWeights = laserEval.stageRWeights;
  msg.laserRunTime = laserEval.runTime;
  msg.lastLaserRunTime = laserEval.lastRunTime;
  msg.laserMeanSqError = laserEval.meanSqError;
  
  msg.pointCloudNumCorrespondences = pointCloudEval.numCorrespondences;
  msg.pointCloudNumObservedPoints = pointCloudEval.numObservedPoints;
  msg.pointCloudStage0Weights = pointCloudEval.stage0Weights;
  msg.pointCloudStageRWeights = pointCloudEval.stageRWeights;
  msg.pointCloudRunTime = pointCloudEval.runTime;
  msg.lastPointCloudRunTime = pointCloudEval.lastRunTime;
  msg.pointCloudMeanSqError = pointCloudEval.meanSqError;
  
  localizationPublisher.publish(msg);
  
  //Publish particles
  vector<Particle2D> particles;
  localization->getParticles(particles);
  particlesMsg.poses.resize(particles.size()); 
  geometry_msgs::Pose particle;
  for(unsigned int i=0; i<particles.size(); i++){
    particle.position.x = particles[i].loc.x;
    particle.position.y = particles[i].loc.y;
    particle.position.z = 0;
    particle.orientation.w = cos(0.5*particles[i].angle);
    particle.orientation.x = 0;
    particle.orientation.y = 0;
    particle.orientation.z = sin(0.5*particles[i].angle);
    particlesMsg.poses[i] = particle;
  }
  particlesPublisher.publish(particlesMsg);
  
  //Publish map to base_footprint tf
  try{
    tf::StampedTransform odomToBaseTf;
    transformListener->lookupTransform("odom","base_footprint",ros::Time(0), odomToBaseTf);
    
    vector2f map_base_trans = curLoc;
    float map_base_rot = curAngle;
    
    vector2f odom_base_trans(odomToBaseTf.getOrigin().x(), odomToBaseTf.getOrigin().y());
    float odom_base_rot = 2.0*atan2(odomToBaseTf.getRotation().z(), odomToBaseTf.getRotation().w());
    
    vector2f base_odom_trans = -odom_base_trans.rotate(-odom_base_rot);
    float base_odom_rot = -odom_base_rot;
    
    vector2f map_odom_trans = map_base_trans + base_odom_trans.rotate(map_base_rot);
    float map_odom_rot = angle_mod(map_base_rot + base_odom_rot);
    
    tf::Transform mapToOdomTf;
    mapToOdomTf.setOrigin(tf::Vector3(V2COMP(map_odom_trans), 0.0));
    mapToOdomTf.setRotation(tf::Quaternion(tf::Vector3(0,0,1),map_odom_rot));
    transformBroadcaster->sendTransform(tf::StampedTransform(mapToOdomTf, ros::Time::now(), "map", "odom"));
  }
  catch (tf::TransformException ex){
    //Do nothing: We'll just try again next time
  }
}

void publishGUI()
{ 
  static double tLast = 0;
  static double publishInterval = 0.016;
  if(debugLevel<0 || GetTimeSec()-tLast<publishInterval)
    return;
  tLast = GetTimeSec();
  ClearGUI();
  //DrawMap();
  guiMsg.robotLocX = curLoc.x;
  guiMsg.robotLocY = curLoc.y;
  guiMsg.robotAngle = curAngle;
  localization->drawDisplay(guiMsg.lines_p1x, guiMsg.lines_p1y, guiMsg.lines_p2x, guiMsg.lines_p2y, guiMsg.lines_col, guiMsg.points_x, guiMsg.points_y, guiMsg.points_col, guiMsg.circles_x, guiMsg.circles_y, guiMsg.circles_col, 1.0);
  //drawPointCloud();
  guiPublisher.publish(guiMsg);
}

void LoadParameters()
{
  WatchFiles watch_files;
  ConfigReader config(ros::package::getPath("cgr_localization").append("/").c_str());
  
  config.init(watch_files);
  
  config.addFile("config/localization_parameters.cfg");
  config.addFile("config/kinect_parameters.cfg");
  
  if(!config.readFiles()){
    printf("Failed to read config\n");
    exit(1);
  }
  
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
  
  {
    ConfigReader::SubTree c(config,"pointCloudParams");
    
    bool error = false;
    error = error || !c.getReal("logObstacleProb", pointCloudParams.logObstacleProb);
    error = error || !c.getReal("logShortHitProb", pointCloudParams.logShortHitProb);
    error = error || !c.getReal("logOutOfRangeProb", pointCloudParams.logOutOfRangeProb);
    error = error || !c.getReal("correlationFactor", pointCloudParams.corelationFactor);
    error = error || !c.getReal("stdDev", pointCloudParams.stdDev);
    error = error || !c.getReal("kernelSize", pointCloudParams.kernelSize);
    error = error || !c.getReal("attractorRange", pointCloudParams.attractorRange);
    error = error || !c.getReal("maxRange", pointCloudParams.maxRange);
    error = error || !c.getReal("minRange", pointCloudParams.minRange);
    
    error = error || !c.getReal("attractorRange", pointCloudParams.attractorRange);
    error = error || !c.getReal("etaAngle", pointCloudParams.etaAngle);
    error = error || !c.getReal("etaLoc", pointCloudParams.etaLoc);
    error = error || !c.getReal("maxAngleGradient", pointCloudParams.maxAngleGradient);
    error = error || !c.getReal("maxLocGradient", pointCloudParams.maxLocGradient);
    error = error || !c.getReal("minCosAngleError", pointCloudParams.minCosAngleError);
    error = error || !c.getInt("numSteps", pointCloudParams.numSteps);
    error = error || !c.getInt("minPoints", pointCloudParams.minPoints);
    error = error || !c.getReal("minRange", pointCloudParams.minRange);
    error = error || !c.getReal("maxRange", pointCloudParams.maxRange);
    error = error || !c.getReal("correspondenceMargin", pointCloudParams.correspondenceMargin);
    error = error || !c.getReal("minRefineFraction", pointCloudParams.minRefineFraction);
    
    if(error){
      printf("Error Loading Point Cloud Parameters!\n");
      exit(2);
    }
  }
  
  planeFilter.setParameters(&kinectDepthCam,filterParams);
}

void InitModels(){
  lidarParams.laserScan = (float*) malloc(lidarParams.numRays*sizeof(float));
  
  filteredPointCloudMsg.header.seq = 0;
  filteredPointCloudMsg.header.frame_id = "base_footprint"; 
  filteredPointCloudMsg.channels.clear();
  
  return;
}

void odometryCallback(const nav_msgs::OdometryConstPtr &msg)
{ 
  static float angle = 0;
  static vector2f loc(0,0);
  static bool initialized=false;
  static double tLast = 0;
  
  if(tLast>msg->header.stamp.toSec())
    initialized = false;
  
  if(!initialized){
    angle = tf::getYaw(msg->pose.pose.orientation);
    loc = vector2f(msg->pose.pose.position.x, msg->pose.pose.position.y);
    initialized = true;
    return;
  }
  
  float newAngle = tf::getYaw(msg->pose.pose.orientation);
  vector2f newLoc(msg->pose.pose.position.x, msg->pose.pose.position.y);
  
  vector2f d = (newLoc-loc).rotate(-angle);
  double dx = d.x;
  double dy = d.y;
  double dtheta = angle_mod(newAngle-angle);
  
  if((sq(dx)+sq(dy))>sq(0.4) || fabs(dtheta)>RAD(40)){
    printf("Odometry out of bounds: x:%7.3f y:%7.3f a:%7.3f\u00b0\n",dx, dy, DEG(dtheta));
    angle = newAngle;
    loc = newLoc;
    return;
  }
  
  if(debugLevel>0) printf("Odometry t:%f x:%7.3f y:%7.3f a:%7.3f\u00b0\n",msg->header.stamp.toSec(), dx, dy, DEG(dtheta));
  
  localization->predict(dx, dy, dtheta, motionParams);
  
  angle = newAngle;
  loc = newLoc;
}

void lidarCallback(const sensor_msgs::LaserScan &msg)
{
  //FunctionTimer ft(__PRETTY_FUNCTION__);
  if(debugLevel>0){
    printf("LIDAR n:%d t:%f noLidar:%d\n",(int) msg.ranges.size(), msg.scan_time, noLidar?1:0);
  }
  
  tf::StampedTransform baseLinkToLaser;
  try{
    transformListener->lookupTransform("base_footprint", msg.header.frame_id, ros::Time(0), baseLinkToLaser);
  }
  catch(tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  tf::Vector3 translationTf = baseLinkToLaser.getOrigin();
  tf::Quaternion rotationTf = baseLinkToLaser.getRotation();
  
  Quaternionf rotQuat3D(rotationTf.w(), rotationTf.x(), rotationTf.y(), rotationTf.z());
  Matrix3f rot3D(rotQuat3D);
  Vector3f trans3D(translationTf.x(), translationTf.y(),translationTf.z());
  
  trans3D = rot3D*trans3D;
  lidarParams.laserToBaseRot = rot3D.topLeftCorner(2,2);
  lidarParams.laserToBaseTrans = trans3D.topLeftCorner(2,1);
  
  bool newLaser = lidarParams.minRange != msg.range_min || 
    lidarParams.maxRange != msg.range_max ||
    lidarParams.minAngle != msg.angle_min || 
    lidarParams.maxAngle != msg.angle_max ||
    lidarParams.angleResolution != msg.angle_increment ||
    lidarParams.numRays != int(msg.ranges.size());
  if(newLaser){
    lidarParams.minRange = msg.range_min;
    lidarParams.maxRange = msg.range_max;
    lidarParams.minAngle = msg.angle_min;
    lidarParams.maxAngle = msg.angle_max;
    lidarParams.angleResolution = msg.angle_increment;
    lidarParams.numRays = int(msg.ranges.size());
    lidarParams.initialize();
  }
  
  if(int(msg.ranges.size()) != lidarParams.numRays){
    TerminalWarning("Incorrect number of Laser Scan rays!");
    printf("received: %d\n",int(msg.ranges.size()));
  }
  
  for(int i=0; i<(int)msg.ranges.size(); i++)
    lidarParams.laserScan[i] = msg.ranges[i];
  
  
  if(!noLidar){
    localization->refineLidar(lidarParams);
    localization->updateLidar(lidarParams, motionParams);
    localization->resample(VectorLocalization2D::LowVarianceResampling);
    localization->computeLocation(curLoc,curAngle);
  }
}


void depthCallback(const sensor_msgs::Image &msg)
{
  //Copy the depth data
  if(debugLevel>0){
    printf("Depth Message t:%f\n", msg.header.stamp.toSec());
  }
  const uint8_t* ptrSrc = msg.data.data();
  uint16_t depth[640*480];
  memcpy(depth, ptrSrc, 640*480*(sizeof(uint16_t)));
  
  if(!usePointCloud)
    return;
  
  tf::StampedTransform baseLinkToKinect;
  try{
    transformListener->lookupTransform("base_footprint", msg.header.frame_id, ros::Time(0), baseLinkToKinect);
  }
  catch(tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  tf::Vector3 translationTf = baseLinkToKinect.getOrigin();
  tf::Quaternion rotationTf = baseLinkToKinect.getRotation();
  Quaternionf rotQuat3D(rotationTf.w(), rotationTf.x(), rotationTf.y(), rotationTf.z());
  Matrix3f rotMatrix(rotQuat3D);
  kinectToRobotTransform.m11 = rotMatrix(0,0);
  kinectToRobotTransform.m12 = rotMatrix(0,1);
  kinectToRobotTransform.m13 = rotMatrix(0,2);
  kinectToRobotTransform.m14 = translationTf.x();
  kinectToRobotTransform.m21 = rotMatrix(1,0);
  kinectToRobotTransform.m22 = rotMatrix(1,1);
  kinectToRobotTransform.m23 = rotMatrix(1,2);
  kinectToRobotTransform.m24 = translationTf.y();
  kinectToRobotTransform.m31 = rotMatrix(2,0);
  kinectToRobotTransform.m32 = rotMatrix(2,1);
  kinectToRobotTransform.m33 = rotMatrix(2,2);
  kinectToRobotTransform.m34 = translationTf.z();
  kinectToRobotTransform.m41 = 0;
  kinectToRobotTransform.m42 = 0;
  kinectToRobotTransform.m43 = 0;
  kinectToRobotTransform.m44 = 1;
  
  //Generate filtered point cloud  
  vector<vector3f> filteredPointCloud;
  vector<vector3f> pointCloudNormals;
  vector<vector3f> outlierCloud;
  vector<vector2i> pixelLocs;
  vector<PlanePolygon> planePolygons;
  
  planeFilter.GenerateFilteredPointCloud(depth, filteredPointCloud, pixelLocs, pointCloudNormals, outlierCloud, planePolygons);
  
  //Transform from kinect coordinates to robot coordinates
  
  for(unsigned int i=0; i<filteredPointCloud.size(); i++){
    filteredPointCloud[i] = filteredPointCloud[i].transform(kinectToRobotTransform);
    pointCloudNormals[i] = pointCloudNormals[i].transform(kinectToRobotTransform);
  }
  

  if(debugLevel>0){
    filteredPointCloudMsg.points.clear();
    filteredPointCloudMsg.header.stamp = ros::Time::now();
    
    vector<geometry_msgs::Point32>* points = &(filteredPointCloudMsg.points);
    geometry_msgs::Point32 p;
    for(int i=0; i<(int)filteredPointCloud.size(); i++){
      p.x = filteredPointCloud[i].x;
      p.y = filteredPointCloud[i].y;
      p.z = filteredPointCloud[i].z;
      points->push_back(p);
    }
    filteredPointCloudPublisher.publish(filteredPointCloudMsg);
  }
  
  //Call particle filter
  {
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
    
    localization->refinePointCloud(pointCloud2D, pointCloudNormals2D, pointCloudParams);
    localization->updatePointCloud(pointCloud2D, pointCloudNormals2D, motionParams, pointCloudParams);
    localization->resample(VectorLocalization2D::LowVarianceResampling);
    localization->computeLocation(curLoc,curAngle);
  }
}

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
  float angle = 2.0*asin(msg.pose.pose.orientation.z);
  vector2f loc(V2COMP(msg.pose.pose.position));
  printf("Initializing CGR localization at %.3f,%.3f, %.2f\u00b0\n",V2COMP(loc), DEG(angle));
  //localization->initialize(numParticles,curMapName.c_str(), loc, angle,sqrt(msg.pose.covariance[0]),sqrt(msg.pose.covariance[35]));
  localization->initialize(numParticles,curMapName.c_str(), loc, angle,0.01,RAD(1.0));
}

int main(int argc, char** argv)
{ 
  //========================= Load Parameters from config file ======================
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
