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
\file    plane_filtering.h
\brief   Interface to PlaneFiltering class, to perform plane filtering of
         3D point clouds from depth images
\author  Joydeep Biswas, (C) 2011
*/
//========================================================================

#ifndef PLANE_FILTERING_H
#define PLANE_FILTERING_H

#include <iostream>
#include <vector>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>

#include "plane_polygon.h"
#include "timer.h"
#include "geometry.h"
#include "grahams_scan.h"
#include <qt4/Qt/qdebug.h>

#include "RGBD.h"

using namespace std;


class PlaneFilter{
  
public:
  /// Parameters for plane filtering
  typedef struct{
    // Parameters for vanilla FSPF
    unsigned int maxPoints;
    unsigned int numSamples;
    unsigned int numLocalSamples;
    unsigned int planeSize;
    float WorldPlaneSize;
    float maxError;
    float minInlierFraction;
    float maxDepthDiff;
    unsigned int numRetries;
    
    // Parameters for polygonization
    bool runPolygonization;
    float minConditionNumber;
    
    //Thresholds for polygon merging
    double maxCosineError;
    float maxPolygonDist;
    float maxOffsetDiff;
    float minVisibilityFraction;
    bool filterOutliers;
  } PlaneFilterParams;
  
private:
  PlaneFilterParams filterParams; 
  uint32_t lastRand;
  
  /// CPU Performance statistics
  AccumulativeTimer planeFilteringTimer;
  /// Sampling Performance statistics
  int numSampledLocations, numPlanarPoints, numNonPlanarPoints;
  
public:
  PlaneFilter();
  //PlaneFilter(DepthCam *_depthCam, PlaneFilterParams &_filterParams);
  PlaneFilter(const RoboCompRGBD::PointSeq &points, PlaneFilterParams &_filterParams);


  //void setDepthCamera(DepthCam *_depthCam){depthCam = _depthCam;}
  //void setFilterParameters(PlaneFilterParams &params){filterParams = params;}
  //void setParameters(DepthCam *_depthCam, PlaneFilterParams &_filterParams){setDepthCamera(_depthCam); setFilterParameters(_filterParams);}
  
  void setParameters(PlaneFilterParams _filterParams){filterParams = _filterParams;}

  
  /// Sample random location in specified window of depth image. Returns true if succesful, along with the sampled point 'p'. Returns false otherwise.
  inline bool sampleLocation(const RoboCompRGBD::PointSeq &points, int& index, int& row, int& col, Vector3f& p, int rMin, int height, int cMin, int width);
  /// Accepts a depth image, returns filtered point cloud + normals, outliers, and all sampled points
  void GenerateFilteredPointCloud(const RoboCompRGBD::PointSeq &points, vector< vector3f >& filteredPointCloud, vector< vector2i >& pixelLocs, vector< vector3f >& pointCloudNormals, vector< vector3f >& outlierCloud, vector< PlanePolygon >& polygons);
  /// Accepts a depth image, returns a complete point cloud with a 3D point for every valid depth pixel
  void GenerateCompletePointCloud(const RoboCompRGBD::PointSeq &points, vector<vector3f> & pointCloud, vector<int> &pixelLocs);
  /// Accepts a depth image, returns a randomly sampled point cloud with at most numPoints
  void GenerateSampledPointCloud(const RoboCompRGBD::PointSeq &points, vector<vector3f> & pointCloud, unsigned int numPoints);
  /// Create a color coded image of same size as depth image, labelling each pixel as belonging to a plane or not
  void GenerateLabelledDepthImage(const RoboCompRGBD::PointSeq &points, int*& labelledDepth, const std::vector< PlanePolygon >& planes);
  /// Returns true of the pixel at index 'ind' corresponds to a plane 
  bool pointIsPlanar(const RoboCompRGBD::PointSeq &points, vector< PlanePolygon >& planes, int ind, float MaxError);
  /// Merges plane polygons belonging to the same plane, and returns a set of unique depth-space planes
  vector<PlanePolygon> findUniqueDepthPlanes(vector< PlanePolygon > planes);
  /// Returns a point cloud from the points extracted from a single raster in the depth image
  void PointCloudFromRaster(const RoboCompRGBD::PointSeq &points, vector< vector3f >& pointCloud, unsigned int raster);
  /// Custom implementation of a linear congruential generator - faster than using glibc's version
  uint32_t lcgRand();
  /// Return performance statistics of plane filtering and depth image sampling
  void getPerformanceStats(double& planeFilteringTime, int& numSampledLocations, int& numPlanarPoints, int& numNonPlanarPoints);
  /// Clear accumulated performance statistics
  void clearPerformanceStats();
};

#endif //PLANE_FILTERING_H
