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
\file    vector_map.h
\brief   C++ Interfaces: VectorMap, LineSection
\author  Joydeep Biswas, (C) 2010
*/
//========================================================================

#include <vector>
#include <fstream>
#include <algorithm>
#include <string>

#include "geometry.h"
#include "line.h"
#include "triangle.h"
#include "terminal_utils.h"
#include "timer.h"
#include <eigen3/Eigen/Eigen>

#ifndef VECTOR_MAP_H
#define VECTOR_MAP_H

using namespace std;

class VectorMap{
  
public:
  typedef struct {
    double a0;
    double a1;
    int index;
    vector2f v0;
    vector2f v1;
  } LineSegment;
  
  vector<line2f> lines;
  
  double vectorScale, vectorOriginX, vectorOriginY, vectorRotation;
  bool vectorFlipY;
  string mapName;
  string mapsFolder;
  
  /// Map Extents 
  float minX, minY, maxX, maxY;
  /// number of meters per cell in visibilityList array
  double visListResolution;
  /// size of the visibilityList array
  unsigned int visListWidth, visListHeight;
  bool preRenderExists;
  double profileTimes[100];
private:
  vector<vector<vector<int> > > visibilityList;
  
public:
  VectorMap(const char* _mapsFolder){lines.clear(); mapsFolder=string(_mapsFolder);}
  VectorMap(const char *name, const char* _mapsFolder, bool usePreRender);
  ~VectorMap();
  
  vector<LineSegment> sortLineSegments(vector2f &loc, vector<line2f> &lines);
  
  /// Get line which intersects first the given ray first
  int getLineCorrespondence(vector2f loc, float angle, float minRange, float maxRange, const std::vector< int >& visibilityList);
  /// Get lines (for each ray) which intersect first the rays starting at angles a0 to a1, at increments of da
  vector<int> getRayToLineCorrespondences(vector2f loc, float angle, float a0, float a1, const std::vector< vector2f > pointCloud, float minRange, float maxRange, bool analytical = false, vector< line2f >* lines = 0);
  /// Convenience function: same as previous, but specified by center angle, angle increment (da), and numRays to scan
  vector<int> getRayToLineCorrespondences(vector2f loc, float a0, float a1, float da, float minRange, float maxRange);
  /// Return intersecting map lines for each ray generated from the provided point cloud (which is assumed to be in local frame)
  vector<int> getRayToLineCorrespondences(vector2f loc, float angle, float da, int numRays, float minRange, float maxRange, bool analytical = false, vector< line2f >* lines = 0);
  /// Get ray cast from vector map at given location and angle, as specified by center angle, angle increment (da), and numRays to scan
  vector<float> getRayCast(vector2f loc, float angle, float da, int numRays, float minRange, float maxRange);
  ///Checks if any part of line2 is occluded by line1 when seen from loc, and if so, line2 is trimmed accordingly, adding sub-lines to sceneLines if necessary
  void trimOcclusion(vector2f &loc, line2f &line1, line2f &line2, vector<line2f> &sceneLines);
  void trimOcclusion2(vector2f& loc_g, line2f& line1, line2f& line2, vector<line2f>& sceneLines);
  /// Get a set of lines which are visible from loc
  vector<int> getSceneLines(vector2f loc, float maxRange);
  /// Load map by name
  bool loadMap(const char* name, bool usePreRender);
  /// Get Visibility list for specified location
  vector<int>* getVisibilityList(float x, float y);
  vector<int>* getVisibilityList(vector2f loc){ return getVisibilityList(loc.x, loc.y); }
  /// Perform an analytical scene render. i.e. Generate a list of lines visible from loc, and the start and end angles subtended by them
  vector<line2f> sceneRender(vector2f loc, float a0=0.0, float a1=M_2PI);
};

#endif //VECTOR_MAP_H
