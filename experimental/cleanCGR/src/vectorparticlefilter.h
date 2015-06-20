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
\file    vectorparticlefilter.h
\brief   C++ Interface: Particle, VectorLocalization
\author  Joydeep Biswas, (C) 2010-2012
*/
//========================================================================

#ifndef VECTORPARTICLEFILTER_H
#define VECTORPARTICLEFILTER_H

#include "stdio.h"
#include <vector>
#include <map>
#include "vector_map.h"
#include <eigen3/Eigen/Dense>
#include "geometry.h"
#include "util.h"
#include "terminal_utils.h"

static const bool EnableProfiling = false;

using namespace std;
using namespace Eigen;
/**
A Particle for 2D localization
**/
class Particle2D {
public:
  vector2f loc, lastLoc;
  float angle;
  float weight;
public:
  Particle2D() {weight = angle = 0.0; loc.zero();}
  Particle2D(float _x, float _y, float _theta, float _w) { loc.set(_x,_y); lastLoc.set(-DBL_MAX,-DBL_MAX); angle = _theta; weight = _w;}
  Particle2D(vector2f _loc, float _theta, float _w) { loc = _loc; angle = _theta; weight = _w;}
  bool operator<(const Particle2D &other) {return weight<other.weight;}
  bool operator>(const Particle2D &other) {return weight>other.weight;}
};

/**
Particle filter for vector localization
**/

class VectorLocalization2D{
  
public:
  typedef struct {
    double tStamp;
    
    //Parameters
    /// Alpha1 = Error in angle per delta in angle
    float Alpha1;
    /// Alpha2 = Error in angle per delta in translation
    float Alpha2;
    /// Alpha3 = Error in translation per delta in translation
    float Alpha3;
    
    float kernelSize;
  } MotionModelParams;
  
  class LidarParams{
    public:
    float* laserScan;
    int numRays;
    float minAngle;
    float maxAngle;
    float angleResolution;
    float minRange;
    float maxRange;
    int minPoints;
    Vector2f laserToBaseTrans;
    Matrix2f laserToBaseRot;
    vector<Vector2f> scanHeadings;
    
    int numSteps;
    float etaAngle;
    float etaLoc;
    float maxLocGradient;
    float maxAngleGradient;
    float attractorRange;
    float minCosAngleError;
    float correspondenceMargin;
    float minRefineFraction;
    
    float logObstacleProb; //Probability of an obstacle
    float logShortHitProb;
    float logOutOfRangeProb;
    float lidarStdDev;
    float correlationFactor;
    
    float kernelSize;
    
    void initialize();
  };
  
  class PointCloudParams{
    public: 
    double tStamp;
    float minRange;
    float maxRange;
    float fieldOfView;
    vector<vector2f> pointCloud;
    vector<vector2f> pointNormals;
    
    //Tunable parameters, MUST be set!
    float logObstacleProb; //Probability of an obstacle
    float logShortHitProb;
    float logOutOfRangeProb;
    float attractorRange;
    float stdDev;
    float corelationFactor;
    
    int numSteps;
    int minPoints;
    float etaAngle;
    float etaLoc;
    float maxLocGradient;
    float maxAngleGradient;
    float minCosAngleError;
    float correspondenceMargin;
    float minRefineFraction;
    
    float kernelSize;
  };
  
  
  typedef struct {
    double lastRunTime;
    double runTime;
    int numObservedPoints;
    int numCorrespondences;
    float stage0Weights;
    float stageRWeights;
    float meanSqError;
  } EvalValues;
  
  enum Resample{
    NaiveResampling,
    LowVarianceResampling,
    SensorResettingResampling,
  };
  
protected:
  //Current state
  VectorMap* currentMap;
  vector2f currentLocation;
  float currentAngle;
  vector2f currentLocStdDev;
  float currentAngleStdDev;
  vector<Particle2D> particles;
  vector<Particle2D> particlesRefined;
  int numParticles;
  vector<float> samplingDensity;
  vector2f lastDistanceMoved;
  float lastAngleTurned;
  
  string mapsFolder;
  vector<VectorMap> maps;
  
  //These are class-wide only so that they can be accessed for debugging purposes
  vector<float> stage0Weights;
  vector<float> stageRWeights;
  vector<Vector2f> gradients;
  vector<Vector2f> points;
  vector<Vector2f> gradients2;
  vector<Vector2f> points2;
  vector<line2f> debugLines;
  vector<int> lineCorrespondences;
  vector2f locCorrectionP0, locCorrectionP1;
  
  //Statistics of performance
  int numUnrefinedParticlesSampled;
  int numRefinedParticlesSampled;
  float refinedImportanceWeights;
  float unrefinedImportanceWeights;
  double refineTime;
  double updateTime;
  EvalValues pointCloudEval;
  EvalValues laserEval;
    
public:
  VectorLocalization2D(const char* _mapsFolder);
  VectorLocalization2D(int _numParticles);
  
  /// Sets Particle Filter LIDAR parameters
  void setParams(MotionModelParams _predictParams, LidarParams _lidarUpdateParams);
  /// Loads All the floor maps listed in atlas.txt
  void loadAtlas();
  /// Initialise arrays, and sets initial location to
  void initialize(int _numParticles, const char* mapName, vector2f loc, float angle, float locationUncertainty = 0.0, float angleUncertainty = 0.0);
  /// Predict step of the particle filter. Samples from the motion model
  void predict(float dx, float dy, float dtheta, const VectorLocalization2D::MotionModelParams& motionParams);
  /// Refine proposal distribution based on LIDAR observations
  void refineLidar(const VectorLocalization2D::LidarParams& lidarParams);
  /// Refine proposal distribution based on Point Cloud observations
  void refinePointCloud(const vector<vector2f>& pointCloud, const vector< vector2f >& pointNormals, const VectorLocalization2D::PointCloudParams& pointCloudParams);
  /// Update distribution based on LIDAR observations
  void updateLidar(const VectorLocalization2D::LidarParams& lidarParams, const VectorLocalization2D::MotionModelParams& motionParams);
  /// Update distribution based on Point Cloud observations
  void updatePointCloud(vector< vector2f >& pointCloud, vector< vector2f >& pointNormals, const VectorLocalization2D::MotionModelParams& motionParams, const VectorLocalization2D::PointCloudParams& pointCloudParams);
  /// Resample distribution
  void resample(Resample type = LowVarianceResampling);
  
  /// Predict particle motion by sampling from the motion model
  void predictParticle(Particle2D& p, float dx, float dy, float dtheta, const VectorLocalization2D::MotionModelParams& motionParams);
  /// Refine a single location hypothesis based on a LIDAR observation
  void refineLocationLidar(vector2f& loc, float& angle, float& initialWeight, float& finalWeight, const VectorLocalization2D::LidarParams& lidarParams, const std::vector< Vector2f >& laserPoints);
  /// Refine a single location hypothesis based on a Point Cloud observation
  void refineLocationPointCloud(vector2f& loc, float& angle, float& initialWeight, float& finalWeight, const vector< vector2f >& pointCloud, const vector< vector2f >& pointNormals, const VectorLocalization2D::PointCloudParams& pointCloudParams);
  
  void computeParticleWeights(vector2f deltaLoc, float deltaAngle, vector2f minLocStdDev, float minAngleStdDev, const VectorLocalization2D::MotionModelParams& motionParams);
  
  /// Attractor function used for refining location hypotheses 
  inline Vector2f attractorFunction(line2f l, Vector2f p, float attractorRange, float margin = 0);
  /// Observation function for a single ray
  inline Vector2f observationFunction(line2f l, Vector2f p);
  /// Gradient based on pointCloud observation
  void getPointCloudGradient(vector2f loc, float angle, vector2f& locGrad, float& angleGrad, const std::vector< vector2f >& pointCloud, const std::vector< vector2f >& pointNormals, float& logWeight, const VectorLocalization2D::PointCloudParams& pointCloudParams, const std::vector< int >& lineCorrespondences, const std::vector< line2f >& lines);
  /// Gradient based on LIDAR observation
  void getLidarGradient(vector2f loc, float angle, vector2f& locGrad, float& angleGrad, float& logWeight, VectorLocalization2D::LidarParams lidarParams, const vector< Vector2f >& laserPoints, const vector<int> & lineCorrespondences, const vector<line2f> &lines);
  /// Observation likelihood based on LIDAR obhservation
  float observationWeightLidar(vector2f loc, float angle, const VectorLocalization2D::LidarParams& lidarParams, const std::vector< Vector2f >& laserPoints);
  /// Observation likelihood based on point cloud obhservation
  float observationWeightPointCloud(vector2f loc, float angle, vector< vector2f >& pointCloud, vector< vector2f >& pointNormals, const PointCloudParams& pointCloudParams);
  /// Probability of specified pose corresponding to motion model
  float motionModelWeight(vector2f loc, float angle, const VectorLocalization2D::MotionModelParams& motionParams);
  /// Set pose with specified uncertainty
  void setLocation(vector2f loc, float angle, const char* map, float locationUncertainty, float angleUncertainty);
  /// Set pose and map with specified uncertainty
  void setLocation(vector2f loc, float angle, float locationUncertainty, float angleUncertainty);
  /// Switch to a different map
  void setMap(const char * map);
  /// Resample particles using low variance resampling
  void lowVarianceResample();
  /// Resample particles using naive resampling
  void naiveResample();
  /// Compute the maximum likelihood location based on particle spread
  void computeLocation(vector2f &loc, float &angle);
  /// Returns the current map name
  const char* getCurrentMapName(){return currentMap->mapName.c_str();}
  /// Creates a particle with the specified properties
  Particle2D createParticle(VectorMap* map, vector2f loc, float angle, float locationUncertainty, float angleUncertainty);
  /// Write to file run statistics about particle distribution
  void saveRunLog(FILE* f);
  /// Write to file riun-time profiling information
  void saveProfilingStats(FILE* f);
  /// Compile lists of drawing primitives that can be visualized for debugging purposes
  void drawDisplay(vector<float> &lines_p1x, vector<float> &lines_p1y, vector<float> &lines_p2x, vector<float> &lines_p2y, vector<uint32_t> &lines_color,
                   vector<float> &points_x, vector<float> &points_y, vector<uint32_t> &points_color, 
                   vector<float> &circles_x, vector<float> &circles_y, vector<uint32_t> &circles_color, float scale=1.0);
  /// Return evaluation values
  void getEvalValues(EvalValues &_laserEval, EvalValues &_pointCloudEval);
  /// Return angle and location uncertainties
  void getUncertainty(float &_angleUnc, float &_locUnc);
  /// Removes duplicate points with the same observation angle and range
  void reducePointCloud(const vector< vector2f >& pointCloud, const vector< vector2f >& pointNormals, vector< vector2f >& reducedPointCloud, vector< vector2f >& reducedPointNormals);
  /// Returns current particles
  void getParticles(vector<Particle2D> &_particles){_particles = particles;}
};

#endif //VECTORPARTICLEFILTER_H

