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
\file    vector_map.cpp
\brief   C++ Implementation: VectorMap, LineSection
\author  Joydeep Biswas, (C) 2010
*/
//========================================================================

#include "vector_map.h"

//============================================================================================

bool ctxVectorMapErrorOccurred = false;

bool VectorMap::loadMap(const char* name, bool usePreRender)
{
  static const bool debug = false;
  
  char vectorFile[4096];
  char renderFile[4096];
  FILE * pFile;
  
  mapName = string(name);
  
  snprintf(vectorFile, 4095, "%s/%s/%s_vector.txt",mapsFolder.c_str(),name,name);
  snprintf(renderFile, 4095, "%s/%s/%s_render.dat",mapsFolder.c_str(),name,name);
  if(debug) printf("Loading map: %s\n",name);
  
  //Read Vector data
  if(debug) printf("Loading vector map from %s\n",vectorFile);
  pFile = fopen (vectorFile,"r");
  if(pFile==NULL){
    char buf[1024];
    snprintf(buf, 1023, "Unable to load vector file %s!",vectorFile);
    TerminalWarning(buf);
    return false;
  }
  float x1,y1,x2,y2;
  minX = minY = FLT_MAX;
  maxX = maxY = -FLT_MAX;
  lines.clear();
  rewind(pFile);
//   printf ("Locale is: %s\n", setlocale(LC_ALL,NULL) );
  // Change locale to american
  setlocale(LC_ALL, "en_US.UTF-8");
//   printf ("Locale is: %s\n", setlocale(LC_ALL,NULL) );
  while(fscanf(pFile,"%f,%f,%f,%f",&x1,&y1,&x2,&y2)==4){
    printf("Line%d: <%f %f> <%f %f>\n",(int)lines.size(),x1,y1,x2,y2);
    // Robocomp to CGR model
    
    float aux = y1;
    y1 = (-1)*x1;
    x1 = aux;
    // ----
    aux = y2;
    y2 = (-1)*x2;
    x2 = aux;
    // ----
    
    minX = min(minX,x1);
    minX = min(minX,x2);
    minY = min(minY,y1);
    minY = min(minY,y2);
    maxX = max(maxX,x1);
    maxX = max(maxX,x2);
    maxY = max(maxY,y1);
    maxY = max(maxY,y2);
    vector2f p0(x1,y1);
    vector2f p1(x2,y2);
    lines.push_back(line2f(p0,p1));
    lines.at(lines.size()-1).calcValues();
  }
  if(debug) printf("%d lines loaded into vector map\n",(int)lines.size());
  if(debug) printf("Extents: %.3f,%.3f : %.3f,%.3f\n",minX, minY, maxX, maxY);
  fclose(pFile);
  
  //Read Pre-Render data
  preRenderExists = false;
  if(usePreRender){
    pFile = fopen(renderFile, "r");
    if(pFile==NULL){
      char buf[1024];
      snprintf(buf, 1023, "Unable to load pre-render file %s!",renderFile);
      TerminalWarning(buf);
    }else{
      bool error = false;
      unsigned int x=0, y=0;
      unsigned int cnt = 0;
      error = fread(&visListWidth,sizeof(unsigned int),1,pFile)!=1;
      error = error || (fread(&visListHeight,sizeof(unsigned int),1,pFile)!=1);
      error = error || (fread(&visListResolution,sizeof(double),1,pFile)!=1);
      if(!error){
        if(debug) printf("Pre-render size: %d x %d, resolution: %.3f\n",visListWidth, visListHeight, visListResolution);
	printf("Pre-render size: %d x %d, resolution: %.3f\n",visListWidth, visListHeight, visListResolution);
        visibilityList.resize(visListWidth);
        for(unsigned int i=0; i<visListWidth; i++)
          visibilityList[i].resize(visListHeight);
      }
      while(cnt<visListHeight*visListWidth && !error){
        size_t size = 0;
        error = fread(&x,sizeof(int),1,pFile)!=1;
        error = error || (fread(&y,sizeof(int),1,pFile)!=1);
        error = error || (fread(&size,sizeof(int),1,pFile)!=1);
        
        if(x<0 || y<0 || x>visListWidth-1 || y>visListHeight-1){
          char msg[4096];
          snprintf(msg, 4095, "Invalid loc (%d,%d) in pre-render file",x,y);
          TerminalWarning(msg);
          error = true;
        }
        visibilityList[x][y].resize(size);
        
        error = error || (fread(visibilityList[x][y].data(),sizeof(int),size,pFile) != size);
        
        if(!error && debug && ((y+1)*100)%visListHeight==0){
          int progress = (y+1)*100/visListHeight;
          printf("\rReading pre-render file... %3d%% ",progress);
          fflush(stdout);
        }
        if(!error)
          cnt++;
      }
      if(error){
        char buf[1024];
        snprintf(buf, 1023, "\nUnable to parse pre-render file %s",renderFile);
        TerminalWarning(buf);
        preRenderExists = false;
      }else{
        if(debug) printf("\nRead %d locations into visibility list\n",cnt);
        preRenderExists = true;
      }
    }
  }
  
  if(debug) printf("Done loading map\n\n");
  return true;
}


VectorMap::VectorMap(const char* name, const char* _mapsFolder, bool usePreRender)
{
  mapsFolder=string(_mapsFolder);
  loadMap(name, usePreRender);
  //initOpenGL();
}

VectorMap::~VectorMap()
{
}

std::vector< int >* VectorMap::getVisibilityList(float x, float y)
{
  int xInd = bound((x-minX)/visListResolution,0.0,visListWidth-1.0);
  int yInd = bound((y-minY)/visListResolution,0.0,visListHeight-1.0);
  return &visibilityList[xInd][yInd];
}

vector<float> VectorMap::getRayCast(vector2f loc, float angle, float da, int numRays, float minRange, float maxRange)
{
  static const bool UsePreRender = true;
  float intervals = numRays - 1.0;
  float a0 = angle - 0.5*intervals*da;
  float a1 = angle + 0.5*intervals*da;
  vector<float> rayCast;
  rayCast.clear();
  vector< int >* visibilityList;
  
  for(float a=a0; a<a1; a += da){
    float ray = maxRange;
    if(preRenderExists && UsePreRender){
      visibilityList = getVisibilityList(loc);
      for(unsigned int i=0; i<visibilityList->size(); i++){
        int lineIndex = visibilityList->at(i);
        float curRay = maxRange;
        if(lines[lineIndex].intersects((vector2f)loc,(float)a)){
          line2f &l = lines[lineIndex];
          //curRay = l.distFromLine1((vector2f)loc,(double)a, false);
          {
            vector2f heading;
            heading.heading(a);
            if(l.intersects(loc,heading,true)){
              float sinTheta = l.Dir().cross(heading.norm());
              curRay = fabs(l.Perp().dot(loc-l.P0())/sinTheta);
            }else{
              curRay = NAN;
            }
          }
          if(curRay<0.0){
            TerminalWarning("Ray Cast Fail!");
            printf("line: %.3f,%.3f : %.3f,%.3f loc:%.3f,%.3f a:%.1f\u00b0 curRay:%f\n",V2COMP(lines[lineIndex].P0()),V2COMP(lines[lineIndex].P1()),V2COMP(loc),DEG(a),curRay);
            fflush(stdout);
            while(1){
              Sleep(0.05);
            }
            //exit(0);
            //alarm(1);
          }
          ray = min(ray, curRay);
        }
      }
    }else{
      /*
      for(unsigned int i=0; i<lines.size(); i++){
        if(lines[i].intersects(loc,a))
          ray = min(ray, lines[i].distFromLine1((vector2f)loc,(double)a, false));
      }
      */
      vector<int> sceneLines = getSceneLines(loc, maxRange);
      for(unsigned int i=0; i<sceneLines.size(); i++){
        if(lines[sceneLines[i]].intersects(loc,a)){
          ray = min(ray, lines[sceneLines[i]].distFromLine1((vector2f)loc,(float)a, false));
        }
      }
      
    }
    rayCast.push_back(ray);
  }
  return rayCast;
}

int VectorMap::getLineCorrespondence(vector2f loc, float angle, float minRange, float maxRange, const std::vector< int >& visibilityList)
{
  vector2f loc1, loc2, dir;
  dir.heading(angle);
  loc1 = loc + minRange*dir;
  loc2 = loc + maxRange*dir;
  
  vector2f p = loc2;
  int bestLine = -1;
  for(unsigned int i=0; i<visibilityList.size(); i++){
    int lineIndex = visibilityList[i];
    if(!lines[lineIndex].intersects(loc1,loc2,false,false,true))
      continue;
    
    vector2f p2 = lines[lineIndex].intersection(loc1,loc2,false,false);
    if( (p2-loc).sqlength()<(p-loc).sqlength()){
      p = p2;
      bestLine = lineIndex;
    }
  }
  return bestLine;
}

vector<int> VectorMap::getRayToLineCorrespondences(vector2f loc, float angle, float a0, float a1, const vector<vector2f> pointCloud, float minRange, float maxRange, bool analytical, vector<line2f> *lines )
{
  //FunctionTimer ft(__FUNCTION__);
  static const bool UsePreRender = true;
  
  vector<int> correspondences;
  correspondences.clear();
  vector<int> locVisibilityList;
  if(UsePreRender && preRenderExists)
    locVisibilityList = *getVisibilityList(loc);
  else
    getSceneLines(loc,maxRange);
  
  if(analytical && lines!=NULL){
    *lines = sceneRender(loc, a0, a1);
    vector<LineSegment> segments = sortLineSegments(loc,*lines);
    float rotMat1[4] = {cos(angle), -sin(angle), sin(angle), cos(angle)};
    vector2f p(0.0,0.0);
    correspondences.resize(pointCloud.size());
    for(uint i=0; i<pointCloud.size(); i++){
      p = pointCloud[i];
      p.set(p.x*rotMat1[0] + p.y*rotMat1[1], p.x*rotMat1[2] + p.y*rotMat1[3]);
      int correspondence = -1;
      
      for(uint j=0; j<segments.size() && correspondence<0; j++){
        if(segments[j].v0.cross(p)>=0.0f && segments[j].v1.cross(p)<=0.0f)
          correspondence = segments[j].index;
      }
      correspondences[i] = correspondence;
    }
  }else{
    for(uint i=0; i<pointCloud.size(); i++){
      float curAngle = angle_mod(pointCloud[i].angle() + angle);
      correspondences.push_back(getLineCorrespondence(loc,curAngle,minRange, maxRange, locVisibilityList));
    }
  }
  return correspondences;
}

vector<int> VectorMap::getRayToLineCorrespondences(vector2f loc, float a0, float a1, float da, float minRange, float maxRange)
{
  //FunctionTimer ft(__PRETTY_FUNCTION__);
  vector<int> correspondences;
  correspondences.clear();
  vector<int> locVisibilityList;
  if(preRenderExists)
    locVisibilityList = *getVisibilityList(loc);
  else
    locVisibilityList = getSceneLines(loc,maxRange);
  for(float a=a0; a<a1; a += da){
    correspondences.push_back(getLineCorrespondence(loc,a,minRange, maxRange, locVisibilityList));
  }
  return correspondences;
}

vector< VectorMap::LineSegment > VectorMap::sortLineSegments(vector2f& loc, vector< line2f >& lines)
{
  static const float eps = RAD(0.001);
  vector< VectorMap::LineSegment > segments;
  segments.clear();
  for(unsigned int i=0; i<lines.size(); i++){
    float a0 = angle_pos((lines[i].P0()-loc).angle());
    float a1 = angle_pos((lines[i].P1()-loc).angle());
    if((lines[i].P0()-loc).cross(lines[i].P1()-loc)<0.0)
      swap(a0,a1);
    LineSegment curSegment;
    curSegment.a0 = a0;
    curSegment.a1 = a1;
    curSegment.index = i;
    curSegment.v0.heading(a0);
    curSegment.v1.heading(a1);
    
    if(segments.size()<1){
      segments.push_back(curSegment);
      continue;
    }
    
    //Special case: line intersects the x axis, so curSegment MUST be the first segment
    if(a0>a1){
      segments.insert(segments.begin(), curSegment);
      continue;
    }
    unsigned int j=0;
    bool found = false;
    for(; j<segments.size()-1 && !found; j++){
      if( segments[j].a1<=a0+eps && segments[j+1].a0>=a1-eps )
        found = true;
    }
    
    if(found && j<segments.size())
      segments.insert(segments.begin()+j,curSegment);
    else
      segments.push_back(curSegment);
  }
  
  if(segments.size()>0){
    if(segments[0].a0>segments[0].a1){
      LineSegment curSegment;
      curSegment.a0 = segments[0].a0;
      curSegment.a1 = M_2PI;
      curSegment.index = segments[0].index;
      segments[0].a0 = 0.0;
      segments.push_back(curSegment);
    }
  }
  
  return segments;
}

vector<int> VectorMap::getRayToLineCorrespondences(vector2f loc, float angle, float da, int numRays, float minRange, float maxRange, bool analytical, vector< line2f > *lines)
{
  //FunctionTimer ft("getRayToLineCorrespondences");
  float intervals = numRays - 1.0;
  float a0 = angle - 0.5*intervals*da;
  float a1 = angle + 0.5*intervals*da;
  
  if(analytical && (lines!=NULL)){
    //FunctionTimer ft("Analytic Render");
    a0 = angle_pos(a0);
    *lines = sceneRender(loc,a0,a1);
    vector< VectorMap::LineSegment > segments = sortLineSegments(loc,*lines);
    vector<int> correspondences;
    correspondences.clear();
    for(int i=0; i<numRays; i++){
      correspondences.push_back(-1);
    }
    int maxScanRays = floor((float)M_2PI/da);
    for(unsigned int i=0; i<segments.size(); i++){
      float aStart = segments[i].a0;
      float aEnd = segments[i].a1;
      if(aEnd<aStart)
        aEnd += M_2PI;
      int index = ceil( (float) (aStart - a0)/da );
      if(index<0)
        index+=maxScanRays;
      for(float a = aStart; a<aEnd; a+=da, index++){
        int j = index%maxScanRays;
        if(j<numRays){
          correspondences[j] = segments[i].index;
        }
      }
    }
    
    return correspondences;
  }else
    return getRayToLineCorrespondences(loc,a0,a1,da,minRange,maxRange);
}

std::vector<int> VectorMap::getSceneLines(vector2f loc, float maxRange)
{
  static const float eps = 1e-6;
  vector<int> linesList, sceneLines;
  vector<line2f> tmpList;
  linesList.clear();
  sceneLines.clear();
  
  for(unsigned int i=0; i<lines.size(); i++){
    if(lines[i].closestDistFromLine(loc, true)<maxRange)
      linesList.push_back(i);
  }
  for(unsigned int i=0; i<linesList.size(); i++){
    line2f curLine = lines[linesList[i]];
    //Check if any part of curLine is unoccluded by present list of lines, as seen from loc
    for(unsigned int j=0;j<linesList.size() && curLine.Length()>=eps; j++){
      if(i==j)
        continue;
      if(lines[linesList[j]].Length()<eps)
        continue;
      trimOcclusion(loc, lines[linesList[j]], curLine,tmpList);
    }
    if( curLine.Length()>eps ){ //At least part of curLine is unoccluded
      sceneLines.push_back(linesList[i]);
    }
  }
  return sceneLines;
}

inline float cross(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2)
{
  return v1.cross(v2).z();
}

static const float eps = 1e-5;

inline bool lineIntersectsLine(const Eigen::Vector3f &l1_p0, const Eigen::Vector3f &l1_p1, const Eigen::Vector3f &l1_dir,
                               const Eigen::Vector3f &l2_p0, const Eigen::Vector3f &l2_p1, const Eigen::Vector3f &l2_dir)
{
  return cross(l1_dir,l2_p0-l1_p0)*cross(l1_dir,l2_p1-l1_p0)<eps && cross(l2_dir,l1_p0-l2_p0)*cross(l2_dir,l1_p1-l2_p0)<eps;
}

inline bool lineIntersectsRay(const Eigen::Vector3f &l1_p0, const Eigen::Vector3f &l1_p1,
                               const Eigen::Vector3f &r_p0, const Eigen::Vector3f &r_dir)
{
  return cross(l1_p0-r_p0,r_dir)*cross(l1_p1-r_p0,r_dir)<eps;
}

inline vector2f tovector2f(const Eigen::Vector3f &v)
{
  return vector2f(v.x(),v.y());
}

inline Eigen::Vector3f lineIntersectionLine(const Eigen::Vector3f &l1_p0, const Eigen::Vector3f &l1_p1, const Eigen::Vector3f &l1_dir,
                                     const Eigen::Vector3f &l2_p0, const Eigen::Vector3f &l2_p1, const Eigen::Vector3f &l2_dir)
{ 
  float den = cross(l2_dir, l1_dir);
  float ua = cross(l1_p0-l2_p0, l1_dir)/den;
  return l2_p0 + ua*l2_dir;
}

inline Eigen::Vector3f lineIntersectionLine(const Eigen::Vector3f &l1_p0, const Eigen::Vector3f &l1_p1,
                                            const Eigen::Vector3f &l2_p0, const Eigen::Vector3f &l2_p1)
{ 
  Eigen::Vector3f l1_dir(l1_p1-l1_p0), l2_dir(l2_p1-l2_p0);
  float den = cross(l2_dir, l1_dir);
  float ua = cross(l1_p0-l2_p0, l1_dir)/den;
  return l2_p0 + ua*l2_dir;
}

void VectorMap::trimOcclusion2(vector2f& loc_g, line2f& line1, line2f& line2, vector< line2f >& sceneLines)
{  
  using namespace Eigen;
  // Checks if any part of line2 is occluded by line1 when seen from loc, and if so, line2 is trimmed accordingly, adding sub-lines to sceneLines if necessary
  static const float eps = 1e-4;
  static const float sqeps = 1e-8;
  if(line1.Length()<eps || line2.Length()<eps)
    return;
  
  Vector3f loc(V2COMP(loc_g),0);
  Vector3f l1_p0(V2COMP(line1.P0()),0);
  Vector3f l1_p1(V2COMP(line1.P1()),0);
  Vector3f l1_dir(l1_p1-l1_p0);
  Vector3f l1_r0(l1_p0-loc);
  Vector3f l1_r1(l1_p1-loc);
  Vector3f l2_p0(V2COMP(line2.P0()),0);
  Vector3f l2_p1(V2COMP(line2.P1()),0);
  Vector3f l2_dir(l2_p1-l2_p0);
  Vector3f l2_r0(l2_p0-loc);
  Vector3f l2_r1(l2_p1-loc);
  
  vector2f l1_p0_g(line1.P0());
  vector2f l1_p1_g(line1.P1());
  vector2f l2_p0_g(line2.P0());
  vector2f l2_p1_g(line2.P1());
  
  //Ensure that r0 vector to r1 vector is in the positive right-handed order
  if( cross(l1_r0,l1_r1)<0.0 ){
    swap(l1_r0,l1_r1);
    swap(l1_p0,l1_p1);
  }
  if( cross(l2_r0,l2_r1)<0.0 ){
    swap(l2_r0,l2_r1);
    swap(l2_p0,l2_p1);
  }
  
  if( (cross(l1_r0, l2_r0)>=0.0 && cross(l1_r1, l2_r0)>=0.0) || (cross(l2_r1, l1_r0)>=0.0 && cross(l2_r1, l2_r1)>=0.0) )
    return; //No Line interaction
    bool intersects, rayOcclusion1, rayOcclusion2;
  
  //completeOcclusion = line1.intersects(loc,l2_p0,false, false, true) && line1.intersects(loc,l2_p1,false, false, true);
  
  //intersects = line2.intersects(line1,false,false,false);
  //rayOcclusion1 = line2.intersects(loc,l1_r0, false); // The semi-infinite ray from loc and passing through line1.p0 intersects line2
  //rayOcclusion2 = line2.intersects(loc,l1_r1, false); // The semi-infinite ray from loc and passing through line1.p1 intersects line2
  
  intersects = lineIntersectsLine(l1_p0,l1_p1,l1_dir, l2_p0,l2_p1,l2_dir);
  rayOcclusion1 = lineIntersectsRay(l2_p0,l2_p1, loc,l1_r0); // The semi-infinite ray from loc and passing through line1.p0 intersects line2
  rayOcclusion2 = lineIntersectsRay(l2_p0,l2_p1, loc,l1_r1); // The semi-infinite ray from loc and passing through line1.p1 intersects line2
  
  Vector3f p, mid;
  if(intersects){
    //line1 and line2 intersect
    //mid = line2.intersection(line1,false,false);
    mid = lineIntersectionLine(l1_p0,l1_p1,l1_dir, l2_p0,l2_p1,l2_dir);
    if( cross((l1_p0-mid), l2_p0-mid) > 0.0){
      //Delete the right hand part of line2
      line2 = line2f(tovector2f(mid),l2_p1_g);
      //line2f l(tovector2f(mid), l2_p0_g);
      //if(l.intersects(loc,l1_r0,false)){
      if(lineIntersectsRay(mid,l2_p0, loc, l1_r0)){
        //p = l.intersection(loc,l1_p0,false, true);
        p = lineIntersectionLine(mid, l2_p0, loc,l1_p0);
        if((l2_p0-p).squaredNorm()>sqeps)  //Part of the right hand part of line2 extends beyond line1, it may still be visible
          sceneLines.push_back(line2f(l2_p0_g, tovector2f(p)));
      }
    }else{
      //Delete the left hand part of line2
      line2 = line2f(l2_p0_g,tovector2f(mid));
      //line2f l(mid, l2_p1);
      //if(l.intersects(loc,l1_r1,false)){
      if(lineIntersectsRay(mid,l2_p1, loc, l1_r1)){
        //p = l.intersection(loc,l1_p1,false, true);
        p = lineIntersectionLine(mid, l2_p0, loc,l1_p1);
        if((l2_p1-p).squaredNorm()>sqeps)  //Part of the left hand part of line2 extends beyond line1, it may still be visible
          sceneLines.push_back(line2f(l2_p1_g, tovector2f(p)));
      }
    }
  }else{ 
    bool completeOcclusion = line1.intersects(line2f(loc_g,l2_p0_g),false, false, true) && line1.intersects(line2f(loc_g,l2_p1_g),false, false, true);
    
    bool occlusion0 = rayOcclusion1 && !line2.intersects(line2f(loc_g,l1_p0_g), false, false, false);  // line1.p0 is in front of, and occludes line2
    bool occlusion1 = rayOcclusion2 && !line2.intersects(line2f(loc_g,l1_p1_g), false, false, false); // line1.p1 is in front of, and occludes line2
    if(completeOcclusion){
      //Trim the line to zero length
      line2 = line2f(0.0,0.0,0.0,0.0);
    }else if(occlusion0 && occlusion1){
      //line2 is partially occluded in the middle by line1. Break up into 2 segments, make line2 one segment, push back the other segment to the sceneLines list
      //mid = line2.intersection(line2f(loc,l1_p0),false, true);
      mid = lineIntersectionLine(l2_p0,l2_p1,l2_dir,loc,l1_p0,l1_r0);
      // newLine2 = the unoccluded part of line2 at its right hand end
      line2f newLine2(l2_p0_g,tovector2f(mid));
      //mid = line2.intersection(line2f(loc_g,l1_p1_g),false, true);
      mid = lineIntersectionLine(l2_p0,l2_p1,l2_dir,loc,l1_p1,l1_r1);
      line2 = newLine2;
      // save the unoccluded part of line2 at its left hand end, if any
      if((mid-l2_p1).squaredNorm()>sqeps)
        sceneLines.push_back(line2f(tovector2f(mid),l2_p1_g));
    }else if(occlusion0){
      //The left hand end of line2 is occluded, trim it
      //vector2f mid = line2.intersection(loc,l1_p0,false, true);
      mid = lineIntersectionLine(l2_p0,l2_p1,l2_dir,loc,l1_p0,l1_r0);
      line2 = line2f(l2_p0_g,tovector2f(mid));
    }else if(occlusion1){
      //The right hand end of line2 is occluded, trim it
      //vector2f mid = line2.intersection(line2f(loc,l1_p1),false, true);
      mid = lineIntersectionLine(l2_p0,l2_p1,l2_dir,loc,l1_p1,l1_r1);
      line2 = line2f(tovector2f(mid),l2_p1_g);
    }
  }
}

void VectorMap::trimOcclusion(vector2f& loc, line2f& line1, line2f& line2, vector< line2f >& sceneLines)
{
  // Checks if any part of line2 is occluded by line1 when seen from loc, and if so, line2 is trimmed accordingly, adding sub-lines to sceneLines if necessary
  static const float eps = 1e-4;
  static const float sqeps = 1e-8;
  if(line1.Length()<eps || line2.Length()<eps)
    return;
  
  vector2f l1_p0 = line1.P0();
  vector2f l1_p1 = line1.P1();
  vector2f l1_r0 = l1_p0-loc;
  vector2f l1_r1 = l1_p1-loc;
  vector2f l2_p0 = line2.P0();
  vector2f l2_p1 = line2.P1();
  vector2f l2_r0 = l2_p0-loc;
  vector2f l2_r1 = l2_p1-loc;
  
  //Ensure that r0 vector to r1 vector is in the positive right-handed order
  if( l1_r0.cross(l1_r1)<0.0 ){
    swap(l1_r0,l1_r1);
    swap(l1_p0,l1_p1);
  }
  if( l2_r0.cross(l2_r1)<0.0 ){
    swap(l2_r0,l2_r1);
    swap(l2_p0,l2_p1);
  }
  
  if( (l1_r0.cross(l2_r0)>=0.0 && l1_r1.cross(l2_r0)>=0.0) || (l2_r1.cross(l1_r0)>=0.0 && l2_r1.cross(l2_r1)>=0.0) )
    return; //No Line interaction
  bool intersects, rayOcclusion1, rayOcclusion2;
  
  //completeOcclusion = line1.intersects(loc,l2_p0,false, false, true) && line1.intersects(loc,l2_p1,false, false, true);
  
  intersects = line2.intersects(line1,false,false,false);
  rayOcclusion1 = line2.intersects(loc,l1_r0, false); // The semi-infinite ray from loc and passing through line1.p0 intersects line2
  rayOcclusion2 = line2.intersects(loc,l1_r1, false); // The semi-infinite ray from loc and passing through line1.p1 intersects line2
  
  vector2f p;
  if(intersects){
    //line1 and line2 intersect
    vector2f mid = line2.intersection(line1,false,false);
    if( (l1_p0-mid).cross(l2_p0-mid) > 0.0){
      //Delete the right hand part of line2
      line2 = line2f(mid,l2_p1);
      line2f l(mid, l2_p0);
      if(l.intersects(loc,l1_r0,false)){
        p = l.intersection(loc,l1_p0,false, true);
        if((l2_p0-p).sqlength()>sqeps)  //Part of the right hand part of line2 extends beyond line1, it may still be visible
          sceneLines.push_back(line2f(l2_p0, p));
      }
    }else{
      //Delete the left hand part of line2
      line2 = line2f(l2_p0,mid);
      line2f l(mid, l2_p1);
      if(l.intersects(loc,l1_r1,false)){
        p = l.intersection(loc,l1_p1,false, true);
        if((l2_p1-p).sqlength()>sqeps)  //Part of the left hand part of line2 extends beyond line1, it may still be visible
          sceneLines.push_back(line2f(l2_p1, p));
      }
    }
  }else{ 
    bool completeOcclusion = line1.intersects(line2f(loc,l2_p0),false, false, true) && line1.intersects(line2f(loc,l2_p1),false, false, true);
    
    bool occlusion0 = rayOcclusion1 && !line2.intersects(line2f(loc,l1_p0), false, false, false);  // line1.p0 is in front of, and occludes line2
    bool occlusion1 = rayOcclusion2 && !line2.intersects(line2f(loc,l1_p1), false, false, false); // line1.p1 is in front of, and occludes line2
    if(completeOcclusion){
      //Trim the line to zero length
      line2 = line2f(0.0,0.0,0.0,0.0);
    }else if(occlusion0 && occlusion1){
      //line2 is partially occluded in the middle by line1. Break up into 2 segments, make line2 one segment, push back the other segment to the sceneLines list
      vector2f mid;
      mid = line2.intersection(line2f(loc,l1_p0),false, true);
      // newLine2 = the unoccluded part of line2 at its right hand end
      line2f newLine2(l2_p0,mid);
      mid = line2.intersection(line2f(loc,l1_p1),false, true);
      line2 = newLine2;
      // save the unoccluded part of line2 at its left hand end, if any
      if((mid-l2_p1).sqlength()>sqeps)
        sceneLines.push_back(line2f(mid,l2_p1));
    }else if(occlusion0){
      //The left hand end of line2 is occluded, trim it
      vector2f mid = line2.intersection(loc,l1_p0,false, true);
      line2 = line2f(l2_p0,mid);
    }else if(occlusion1){
      //The right hand end of line2 is occluded, trim it
      vector2f mid = line2.intersection(line2f(loc,l1_p1),false, true);
      line2 = line2f(mid,l2_p1);
    }
  }
}

vector<line2f> VectorMap::sceneRender(vector2f loc, float a0, float a1)
{
  //FunctionTimer ft("Scene Render");
  static const float eps = 0.001;
//   static const float MinRange = 0.03;
  static const float MaxRange = 5.0;
  static const unsigned int MaxLines = 200;
  
  vector2f leftMargin, rightMargin;
  rightMargin.heading(a0);
  leftMargin.heading(a1);
//   bool obtuseView = rightMargin.cross(leftMargin)<=0.0;
  
  vector<line2f> scene, sceneCleaned;
  vector<line2f> linesList;
  vector<int>* locVisibilityList;
  
  scene.clear();
  sceneCleaned.clear();
  linesList.clear();
  
  if(preRenderExists){
    locVisibilityList = getVisibilityList(loc);
  }else{
    locVisibilityList = new vector<int>;
    *locVisibilityList = getSceneLines(loc,MaxRange);
  }
  for(unsigned int i=0; i<locVisibilityList->size(); i++){
    linesList.push_back(lines[locVisibilityList->at(i)]);
  }
  unsigned int i, j;
  for(i=0; i<linesList.size() && i<MaxLines; i++){
    line2f curLine = linesList[i];
    //Check if any part of curLine is unoccluded by present list of lines, as seen from loc
    for(j=0;j<scene.size() && curLine.Length()>=eps; j++){
      if(scene[j].Length()<eps)
        continue;
      trimOcclusion(loc, scene[j], curLine,linesList);
    }
    
    if( curLine.Length()>eps ){ //At least part of curLine is unoccluded
      for(j=0; j<scene.size(); j++){
        if(scene[j].Length()<eps)
          continue;
        trimOcclusion(loc, curLine, scene[j],linesList);
      }
      //add the visible part of curLine
      scene.push_back(curLine);
    }
  }
  
  if(linesList.size()>=MaxLines){
    char buf[2048];
    sprintf(buf,"Runaway Analytic Scene Render at %.30f,%.30f, %.3f : %.3f\u00b0",V2COMP(loc),DEG(a0),DEG(a1));
    TerminalWarning(buf);
  }
  for(i=0; i<scene.size(); i++){
    if(scene[i].Length()>eps)
      sceneCleaned.push_back(scene[i]);
  }
  return sceneCleaned;
}
