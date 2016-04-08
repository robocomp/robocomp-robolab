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
\file    kinect_main.cpp
\brief   Publish 3D point cloud from the Kinect
\author  Joydeep Biswas, (C) 2010
*/
//========================================================================

#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <stdio.h>

#include "terminal_utils.h"
#include "proghelp.h"
#include "sys/time.h"
#include <ros/ros.h>
#include "timer.h"
#include <vector>

#include "popt_pp.h"
#include <libusb.h>
#include "libfreenect.h"
#include <pthread.h>
#include <math.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "plane_filtering.h"
#include "configreader.h"
#include "geometry.h"

using namespace std;

bool run = true;

ros::NodeHandle *n;
KinectRawDepthCam kinectDepthCam;

sensor_msgs::Image rawDepthImgMsg;
sensor_msgs::Image depthImgMsg;
sensor_msgs::Image rgbImgMsg;

//ROS publisher
ros::Publisher rgbImagePublisher;
ros::Publisher depthImagePublisher;
ros::Publisher processedDepthImagePublisher;

//Run modes
bool captureReady = false;
bool runColor = false;
int debugLevel = -1;

//Device data
int kinectDeviceNumber = 0;

//Kinect pose
vector3f kinectLoc;
float xRot, yRot, zRot;

void rgb_cb(freenect_device *dev, freenect_pixel *rgb, uint32_t timestamp);

WatchFiles watch_files;
ConfigReader config("./");

void LoadParameters()
{
  if(!config.readFiles()){
    printf("Failed to read config\n");
    exit(1);
  }
  
  ConfigReader::SubTree c(config,"KinectParameters");
  
  unsigned int maxDepthVal=1024;
  bool error = false;
  error = error || !c.getReal("f",kinectDepthCam.f);
  error = error || !c.getReal("fovH",kinectDepthCam.fovH);
  error = error || !c.getReal("fovV",kinectDepthCam.fovV);
  error = error || !c.getInt("width",kinectDepthCam.width);
  error = error || !c.getInt("height",kinectDepthCam.height);
  error = error || !c.getUInt("maxDepthVal",maxDepthVal);
  kinectDepthCam.maxDepthVal = maxDepthVal;
  
  float xRot, yRot, zRot;
  error = error || !c.getVec3f("loc",kinectLoc);
  error = error || !c.getReal("xRot",xRot);
  error = error || !c.getReal("yRot",yRot);
  error = error || !c.getReal("zRot",zRot);
  
  if(error){
    printf("Error Loading Kinect Parameters!\n");
    exit(2);
  }
}

void InitMsgs()
{
  rgbImgMsg.header.seq = 0;
  rgbImgMsg.header.frame_id = "kinect";
  rgbImgMsg.height = 480;
  rgbImgMsg.width = 640;
  rgbImgMsg.encoding = sensor_msgs::image_encodings::RGB8;
  rgbImgMsg.is_bigendian = 0;
  rgbImgMsg.step = 640*3;
  rgbImgMsg.data.resize(640*480*3);
  
  rawDepthImgMsg.header.seq = 0;
  rawDepthImgMsg.header.frame_id = "kinect";
  rawDepthImgMsg.height = 480;
  rawDepthImgMsg.width = 640;
  rawDepthImgMsg.encoding = sensor_msgs::image_encodings::MONO16;
  rawDepthImgMsg.is_bigendian = 0;
  rawDepthImgMsg.step = 640*2;
  rawDepthImgMsg.data.resize(640*480*2);
  
  depthImgMsg.header.seq = 0;
  depthImgMsg.header.frame_id = "kinect";
  depthImgMsg.height = 480;
  depthImgMsg.width = 640;
  depthImgMsg.encoding = sensor_msgs::image_encodings::MONO8;
  depthImgMsg.is_bigendian = 0;
  depthImgMsg.step = 640;
  depthImgMsg.data.resize(640*480);
  
}

void rgbMsgCallback(const sensor_msgs::ImageConstPtr& msg)
{
  static const bool debug = false;
  if(debug) printf("Kinect RGB message received, t=%f\n",msg->header.stamp.toSec());
  rgb_cb(NULL, (freenect_pixel*) msg->data.data(), 0);  
}

void PublishProcessedDepth(freenect_depth *depth)
{
  static const double MaxDepth = 4.0;
  depthImgMsg.header.seq++;
  depthImgMsg.header.stamp = ros::Time::now();
  double val;
  for(int i=0; i<640*480; i++){
    val = bound(1.0/(-0.00307110156*double(depth[i])+3.3309495)/MaxDepth,0.0,1.0);
    depthImgMsg.data[i] = floor(255.0*val);
  }
  processedDepthImagePublisher.publish(depthImgMsg);
}

void depth_cb(freenect_device *dev, freenect_depth *depth, uint32_t timestamp)
{ 
  if(!captureReady)
    return;
  
  uint8_t* ptrDest = rawDepthImgMsg.data.data();
  memcpy(ptrDest, depth, FREENECT_DEPTH_SIZE);
  
  if(debugLevel>0)
    PublishProcessedDepth(depth);
  
  rawDepthImgMsg.header.seq++;
  rawDepthImgMsg.header.stamp = ros::Time::now();
  depthImagePublisher.publish(rawDepthImgMsg);
}

void rgb_cb(freenect_device *dev, freenect_pixel *rgb, uint32_t timestamp)
{
  if(!captureReady)
    return;
  
  uint8_t* ptrDest = rgbImgMsg.data.data();
  memcpy(ptrDest, rgb, FREENECT_RGB_SIZE);
  rgbImgMsg.header.seq++;
  rgbImgMsg.header.stamp = ros::Time::now();
  rgbImagePublisher.publish(rgbImgMsg);
}

int kinect_main()
{
  freenect_context *f_ctx;
  freenect_device *f_dev;
    
  if (freenect_init(&f_ctx, NULL) < 0) {
    printf("freenect_init() failed\n");
    return 1;
  }
  
  int nr_devices = freenect_num_devices (f_ctx);
  if(debugLevel>=0) printf ("Number of devices found: %d\n", nr_devices);
  
  if (nr_devices < 1){
    printf("No Kinect connected!\n");
    return 1;
  }
  
  if (freenect_open_device(f_ctx, &f_dev, kinectDeviceNumber) < 0) {
    printf("Could not open device\n");
    return 1;
  }
  
  freenect_set_led(f_dev,LED_GREEN);
  freenect_set_depth_callback(f_dev, depth_cb);
  
  freenect_set_rgb_callback(f_dev, rgb_cb);
  freenect_set_rgb_format(f_dev, FREENECT_FORMAT_RGB);
  freenect_set_depth_format(f_dev, FREENECT_FORMAT_11_BIT);
  
  if(debugLevel>=0) printf("Starting depth capture... "); fflush(stdout);
  freenect_start_depth(f_dev);
  if(debugLevel>=0) printf("Ready.\n"); fflush(stdout);
 
  
  if(runColor){
    if(debugLevel>=0) printf("Starting RGB capture... "); fflush(stdout);
    freenect_start_rgb(f_dev);
    if(debugLevel>=0) printf("Ready.\n"); fflush(stdout);
  }
  
  captureReady = true;
  
  while(run && freenect_process_events(f_ctx) >= 0 && ros::ok()){
    
    if(watch_files.getEvents() != 0){
      printf("Reloading parameters.\n");
      LoadParameters();
      watch_files.clearEvents();
    }
    Sleep(0.01);
  }
  
  //freenect_stop_depth(f_dev);
  //freenect_stop_rgb(f_dev);
  
  if(debugLevel>=0) printf("done!\n");
  freenect_set_led(f_dev,LED_BLINK_GREEN);
  
  return 0;
}

int main(int argc, char **argv) {
  
  ColourTerminal(TerminalUtils::TERMINAL_COL_WHITE,TerminalUtils::TERMINAL_COL_BLACK,TerminalUtils::TERMINAL_ATTR_BRIGHT);
  printf("\nKinect module\n\n");
  ResetTerminal();
  
  // option table
  static struct poptOption options[] = {
    { "debug"               , 'd', POPT_ARG_INT   , &debugLevel,         0, "Debug Level"             , "NUM" },
    { "device-num"          , 'n', POPT_ARG_INT   , &kinectDeviceNumber ,0, "Kinect device number"    , "NUM" },
    { "color"               , 'c', POPT_ARG_NONE  , &runColor           ,0, "Run Color Camera"        , "NONE"},
    
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };
  // parse options
  POpt popt(NULL,argc,(const char**)argv,options,0);
  int c;
  while((c = popt.getNextOpt()) >= 0){
  }
  
  config.init(watch_files);
  config.addFile("config/kinect_parameters.cfg");
  LoadParameters();
  InitMsgs();
  
  ros::init(argc, argv, "Kinect_Module", ros::init_options::NoSigintHandler);
  n = new ros::NodeHandle();
  rgbImagePublisher = n->advertise<sensor_msgs::Image>("Kinect/RGB", 1);
  depthImagePublisher = n->advertise<sensor_msgs::Image>("Kinect/Depth", 1);
  processedDepthImagePublisher = n->advertise<sensor_msgs::Image>("Kinect/ProcessedDepth", 1);
  
  // main program initialization
  InitHandleStop(&run);
  
  return kinect_main();
  
  if(debugLevel>=0) printf("Closing.\n");
  
  return(0);
}
