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
\file    vector_display_main.cpp
\brief   A graphical vector localization vizualizer 
\author  Joydeep Biswas, (C) 2010
*/
//========================================================================

#include "stdio.h"

#include <QtGui/QApplication>
#include <QObject>
#include <QThread>
#include <vector>

#include "vector_display.h"
#include "proghelp.h"
#include "timer.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "popt_pp.h"
#include "cgr_localization/DisplayMsg.h"
#include "cgr_localization/LocalizationInterfaceSrv.h"
#include "cgr_localization/LocalizationMsg.h"
#include "sensor_msgs/LaserScan.h"

#include "util.h"
#include "geometry.h"
#include "terminal_utils.h"
#include "vector_map.h"

using namespace std;
using namespace ros;
using namespace cgr_localization;

ros::NodeHandle *n;

bool runApp = true;
bool testMode = false;
bool saveLocs = false;
bool saveOrientations = false;
bool liveView = false;
bool persistentDisplay = true;

VectorDisplay* display;
QApplication* app;
ros::ServiceClient client;
ros::ServiceClient localizationClient;
char* map_name;

void callback(vector2d loc,vector2d loc2,double orientation,int type)
{
  static const bool debug = false;
  static FILE* fid=NULL;
  if(saveLocs && fid==NULL)
    fid = fopen("savedLocs.txt","aw");
  if(saveLocs){
    if(saveOrientations){
      fprintf(fid, "%f,%f,%f\n",V2COMP(loc),(loc2-loc).angle());
      printf("Saved location: %f,%f,%f\n",V2COMP(loc),(loc2-loc).angle());
    }else{
      fprintf(fid, "%f,%f,%f,%f\n",V2COMP(loc),V2COMP(loc2));
      printf("Saved location: %f,%f,%f,%f\n",V2COMP(loc),V2COMP(loc2));
    }
    
  }
  LocalizationInterfaceSrv srv;
  srv.request.loc_x = loc.x;
  srv.request.loc_y = loc.y;
  srv.request.orientation = orientation;
  srv.request.map = map_name;
  switch(type){
    case 3:{
      //Set Position
      if(debug) printf("SetPosition: %7.3f,%7.3f %6.1f\u00b0\n",V2COMP(loc),DEG(orientation));
      client = localizationClient;
    }break;
    case 2:{
      //Set Target
      if(debug) printf("SetTarget: %7.3f,%7.3f %6.1f\u00b0\n",V2COMP(loc),DEG(orientation));
      //client = navigationClient;
    }break;
  }
  if(type==2 || type==3){
    if(client.call(srv)){
      
    }else{
      printf("Failed to send command!\n");
    }
  }
}

class MyThread : public QThread
{
  //Q_OBJECT
  string mapsFolder;
  VectorMap vectorMap;
  vector<vector2d> pathPlan;
  vector2d robotLoc;
  double robotAngle;
  sensor_msgs::LaserScan laserScanMsg;
  vector<DisplayMsg> displayMsgs;
  vector<string> displayProviders;
  double tPathPlan, tDisplay, tLaser;
  
  vector<line2f> lines;
  vector<vector2f> points;
  vector<vector2f> circles;
  vector<VectorDisplay::Color> circleColors;
  vector<VectorDisplay::Color> lineColors;
  vector<VectorDisplay::Color> pointColors;
  
public:
  void drawMap(vector<line2f> &lines, vector<VectorDisplay::Color> &lineColors)
  {
    int numLines = vectorMap.lines.size();
    for(int i=0; i<numLines; i++){
      lines.push_back(line2f(vectorMap.lines[i].P0().x,vectorMap.lines[i].P0().y,
                              vectorMap.lines[i].P1().x,vectorMap.lines[i].P1().y));
      lineColors.push_back(VectorDisplay::Color(0.32, 0.49, 0.91));
    }
  }
  
  void localizationCallback(const LocalizationMsg& msg)
  {
    static const bool debug = false;
    if(debug) printf("Localization update: %f,%f, %f\u00b0, %s\n",msg.x,msg.y,DEG(msg.angle),msg.map.c_str());
    robotLoc = vector2d(msg.x, msg.y);
    robotAngle = msg.angle;
    snprintf(map_name, 4095, "%s",msg.map.c_str());
    if(msg.map.compare(vectorMap.mapName)!=0){
      vectorMap.loadMap(msg.map.c_str(), false);
    }
  }
  
  void laserCallback(const sensor_msgs::LaserScan& msg)
  {
    laserScanMsg = msg;
    tLaser = GetTimeSec();
  }
    
  void statusCallback(const ros::MessageEvent<DisplayMsg const>& msgEvent)
  { 
    static const bool debug = false;
    const DisplayMsgConstPtr &msg = msgEvent.getConstMessage();
    bool duplicate = false;
    unsigned int i=0;
    for(;i<displayProviders.size() && !duplicate; i++){
      if(displayProviders[i].compare(msgEvent.getPublisherName())==0)
        duplicate = true;
    }
    if(duplicate){
      i--;
      displayMsgs[i] = *msg;
      if(debug) printf("Duplicate message from %s\n",displayProviders[i].c_str());
    }else{
      displayMsgs.push_back(*msg);
      displayProviders.push_back(msgEvent.getPublisherName());
      if(debug) printf("New message from %s\n",displayProviders[i].c_str());
    }
  }
  
  void compileDisplay()
  {
    static double tLast = 0.0;
    static const double MessageTimeout = 1.0;
    static const int LidarPointColor = 0xF0761F;
    static const bool debug = false;
    if(debug) printf("GUI updated!\n");
    
    if(GetTimeSec()-tLast<0.016)
      return;
    tLast = GetTimeSec();
    
    lines.clear();
    points.clear();
    circles.clear();
    circleColors.clear();
    lineColors.clear();
    pointColors.clear();
    
    drawMap(lines, lineColors);
    for(unsigned int j=0; j<displayMsgs.size(); j++){
      DisplayMsg displayMsg = displayMsgs[j];
      if(GetTimeSec()-tDisplay<MessageTimeout || persistentDisplay){
        unsigned int numLines = min(min((int)displayMsg.lines_p1x.size(),(int)displayMsg.lines_p1y.size()),min((int)displayMsg.lines_p2x.size(),(int)displayMsg.lines_p2y.size()));
        for(unsigned int i=0; i<numLines; i++){
          lines.push_back(line2f(displayMsg.lines_p1x[i],displayMsg.lines_p1y[i],displayMsg.lines_p2x[i],displayMsg.lines_p2y[i]));
          if(i<displayMsg.lines_col.size())
            lineColors.push_back(displayMsg.lines_col[i]);
        }
        unsigned int numPoints = min(displayMsg.points_x.size(),displayMsg.points_y.size());
        for(unsigned int i=0; i<numPoints; i++){
          points.push_back(vector2f(displayMsg.points_x[i],displayMsg.points_y[i]));
          if(i<displayMsg.points_col.size())
            pointColors.push_back(VectorDisplay::Color(displayMsg.points_col[i]));
        }
        unsigned int numCircles = min(displayMsg.circles_x.size(),displayMsg.circles_y.size());
        for(unsigned int i=0; i<numCircles; i++){
          circles.push_back(vector2f(displayMsg.circles_x[i],displayMsg.circles_y[i]));
          if(i<displayMsg.circles_col.size())
            circleColors.push_back(VectorDisplay::Color(displayMsg.circles_col[i]));
        }
      }
    }
    if(debug){
      printf("lines: %d points: %d circles: %d\n",int(lines.size()), int(points.size()), int(circles.size()));
    }
    //displayMsgs.clear();
    //displayProviders.clear();
    
    if((GetTimeSec()-tLaser<MessageTimeout || persistentDisplay) && liveView){
      unsigned int i=0;
      float a=0.0;
      vector2f laserLoc(0.145,0.0), p(0.0,0.0);
      laserLoc = vector2f(V2COMP(robotLoc)) + laserLoc.rotate(robotAngle);
      for(i=0, a = robotAngle + laserScanMsg.angle_min; i<laserScanMsg.ranges.size(); i++, a+=laserScanMsg.angle_increment){
        if(laserScanMsg.ranges[i]<=laserScanMsg.range_min || laserScanMsg.ranges[i]>=laserScanMsg.range_max)
          continue;
        p.heading(a);
        p = p*laserScanMsg.ranges[i]+laserLoc;
        points.push_back(p);
        pointColors.push_back(LidarPointColor);
      }
    }
    
    if(GetTimeSec()-tPathPlan<MessageTimeout || persistentDisplay){
      if(pathPlan.size()>0){
        lines.push_back(line2f(pathPlan[0].x, pathPlan[0].y, robotLoc.x, robotLoc.y));
        lineColors.push_back(0x00FF00);
        points.push_back(vector2f(V2COMP(pathPlan[0])));
        pointColors.push_back(0xFF0000);
      }
      for(int i=0; i<int(pathPlan.size())-1; i++){
        lines.push_back(line2f(pathPlan[i].x, pathPlan[i].y, pathPlan[i+1].x, pathPlan[i+1].y));
        lineColors.push_back(0x00FF00);
        points.push_back(vector2f(V2COMP(pathPlan[i])));
        points.push_back(vector2f(V2COMP(pathPlan[i+1])));
        pointColors.push_back(0xFF0000);
        pointColors.push_back(0xFF0000);
      }
    }
    
    display->updateDisplay(robotLoc,robotAngle, 100.0, lines, points, circles, lineColors, pointColors, circleColors);
  }
  
protected:
  void run()
  {
    static const bool debug = false;
    
    if(testMode){
      while(runApp && ros::ok()){
        float scale = 0.005;
        int numLines = 10;
        int numPoints = 800;
        vector<line2f> lines;
        vector<vector2f> points;
        double dTheta = 2.0*M_PI/double(numLines);
        vector2d offset(1000.0,0.0);
        double theta = 0.0;
        double omega = RAD(30.0);
        for(int i=0; i<numLines; i++){
          vector2d p(1000.0,0.0), p1,p2;
          p1 = p.rotate(theta+omega*GetTimeSec());
          p2 = p.rotate(theta+dTheta+omega*GetTimeSec());
          p1 *= scale;
          p2 *= scale;
          lines.push_back(line2f(p1.x,p1.y,p2.x,p2.y));
          theta += dTheta;
        }
        dTheta = 2.0*M_PI/double(numPoints);
        theta = 0.0;
        for(int i=0;i<numPoints;i++){
          vector2d p(3500.0,0.0);
          p = p*max(0.0,1.1+sin(sin(2.0*theta)*M_PI))/2.0;
          p = p.rotate(theta+omega*GetTimeSec())+offset;
          p *= scale;
          points.push_back(vector2f(p.x,p.y));
          theta += dTheta;
        }
        display->updateLines(lines);
        display->updatePoints(points);
        Sleep(0.016);
      }
      if(debug) printf("Terminating testMode thread. runApp:%d ok():%d\n",runApp?1:0,ros::ok()?1:0);
    }else{
      
      vectorMap = VectorMap(map_name,mapsFolder.c_str(), false);
      
      ros::Subscriber guiSub;
      ros::Subscriber statusSub;
      ros::Subscriber laserSub;
      ros::Subscriber localizationSub;
      
      laserSub = n->subscribe("laser", 1, &MyThread::laserCallback, this);
      localizationSub = n->subscribe("localization", 1, &MyThread::localizationCallback, this);
      guiSub = n->subscribe("localization_gui", 1, &MyThread::statusCallback, this);
      
      drawMap(lines, lineColors);
      display->updateLines(lines, lineColors);
      
      while(runApp && ros::ok()){
        spinOnce();
        compileDisplay();
        Sleep(0.001);
      }
      if(debug) printf("Terminating thread. runApp:%d ok():%d\n",runApp?1:0,ros::ok()?1:0);
    }
    app->quit();
  }
  public:
    MyThread(const char* _mapsFolder, QObject* parent = 0) : vectorMap(_mapsFolder)
    { 
      mapsFolder = string(_mapsFolder); 
      pathPlan.clear(); 
    }
    ~MyThread(){}
};


int main(int argc, char *argv[])
{
  static const bool debug = false;
  ColourTerminal(TerminalUtils::TERMINAL_COL_GREEN,TerminalUtils::TERMINAL_COL_BLACK,TerminalUtils::TERMINAL_ATTR_BRIGHT);
  printf("\nVector LIDAR Localization GUI\n\n");
  ResetTerminal();
  
  if(debug) printf("Starting up...\n");
  
  ros::init(argc, argv, "vector_LIDAR_GUI");
  map_name = (char*) malloc(4096);
  snprintf(map_name, 4095, "GHC7");
  // option table
  static struct poptOption options[] = {
    { "test-mode",        't', POPT_ARG_NONE ,   &testMode,           0, "Test Drawing Capabilities", "NONE"},
    { "map-name",         'm', POPT_ARG_STRING , &map_name,           0, "Map name",                  "STRING"},
    { "save-locs",        's', POPT_ARG_NONE,    &saveLocs,           0, "Save Locations",            "NONE"},
    { "live-view",        'l', POPT_ARG_NONE,    &liveView,           0, "Live View of Robot",        "NONE"},
    { "persistent-display",'p', POPT_ARG_NONE,   &persistentDisplay,  0, "Persistent Display",        "NONE"},
    { "save-orientation",  'S', POPT_ARG_NONE,   &saveOrientations,   0, "Save Orientations",         "NONE"},
    
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };
  // parse options
  POpt popt(NULL,argc,(const char**)argv,options,0);
  int c;
  while((c = popt.getNextOpt()) >= 0){
  }
  if(saveOrientations)
    saveLocs = true;
  
  app = new QApplication(argc, argv);
  display = new VectorDisplay();
  
  display->setCallback(&callback);
  
  n = new ros::NodeHandle();
  localizationClient = n->serviceClient<LocalizationInterfaceSrv>("localization_interface");
  
  InitHandleStop(&runApp);
  display->show();
  MyThread thread(ros::package::getPath("cgr_localization").append("/maps/").c_str());
  thread.start();
  int retVal = app->exec();
  runApp = false;
  if(debug) printf("Waiting for thread termination... ");
  thread.wait();
  if(debug) printf("Done. Bye Bye!\n");
  return retVal;
}
