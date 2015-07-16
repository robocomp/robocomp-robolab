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

//file    vectorparticlefilter.h
//brief   C++ Implementation: Particle, VectorLocalization
//author  Joydeep Biswas, (C) 2010-2012
//========================================================================

#include "vectorparticlefilter.h"
#include <QtGlobal>
#include <list>

static const bool UseAnalyticRender = true;


inline float eigenCross(const Vector2f &v1, const Vector2f &v2)
{
  return v1.dot(Vector2f(v2.y(),-v2.x()));
}

void VectorLocalization2D::LidarParams::initialize()
{
  //float a0 = -0.5*angleResolution*float(numRays);
  scanHeadings.resize(numRays);
  int i=0;
  for(float a=minAngle; a<maxAngle; a+=angleResolution){
	scanHeadings[i] = Vector2f(-sin(a),cos(a));		//with our axis reference
    i++;
  }
  //TODO delete laserScan
  laserScan = (float*) malloc(numRays*sizeof(float));
}

VectorLocalization2D::VectorLocalization2D(const char* _mapsFolder)
{
  mapsFolder = string(_mapsFolder);
  loadAtlas();
  numParticles = 0;
  particles.clear();
  locCorrectionP0.zero();
  locCorrectionP1.zero();
}

VectorLocalization2D::VectorLocalization2D(int _numParticles)
{
  loadAtlas();
  numParticles = _numParticles;
  particles.resize(_numParticles);
  stage0Weights.resize(_numParticles);
  stageRWeights.resize(_numParticles);
}

void VectorLocalization2D::loadAtlas()
{
  static const bool debug = true;
  string atlasFile = mapsFolder + "/atlas.txt";
  cout <<atlasFile << endl;
  FILE* fid = fopen(atlasFile.c_str(),"r");
  if(fid==NULL){
    TerminalWarning("Unable to load Atlas!");
    return;
  }
  char mapName[4096];
  int mapNum;
  if(debug) printf("Loading Atlas...\n");
  maps.clear();
  while(fscanf(fid,"%d %s\n",&mapNum,mapName)==2){
    if(debug) printf("Loading map %s\n",mapName);
    maps.push_back(VectorMap(mapName,mapsFolder.c_str(),true));
  }
  if(maps.size()>0)
    currentMap = &maps[0];
  if(debug) printf("Done Loading Atlas.\n");
}

Particle2D VectorLocalization2D::createParticle(VectorMap* map, vector2f loc, float angle, float locationUncertainty, float angleUncertainty)
{
  return Particle2D(randn(locationUncertainty,loc.x), randn(locationUncertainty,loc.y), randn(angleUncertainty,angle),1.0);
}

void VectorLocalization2D::setLocation(vector2f loc, float angle, const char* map, float locationUncertainty, float angleUncertainty)
{
  for(unsigned int i=0; i<particles.size(); i++){
    particles[i].loc = vector2f(randn(locationUncertainty, loc.x), randn(locationUncertainty, loc.y));
    particles[i].angle = randn(angleUncertainty, angle);
  }

  bool found = false;
  int mapIndex=0;
  for(unsigned int i=0; !found && i<maps.size(); i++){
    if(maps[i].mapName.compare(map)==0){
      mapIndex = i;
      found = true;
    }
  }
  if(found){
    currentMap = &maps[mapIndex];
  }else{
    char buf[4096];
    snprintf(buf, 4095, "Unknown map: \"%s\"",map);
    TerminalWarning(buf);
  }  
}

void VectorLocalization2D::setLocation(vector2f loc, float angle, float locationUncertainty, float angleUncertainty)
{
  for(unsigned int i=0; i<particles.size(); i++){
    particles[i].loc = vector2f(randn(locationUncertainty, loc.x), randn(locationUncertainty, loc.y));
    particles[i].angle = randn(angleUncertainty, angle);
  }
}

void VectorLocalization2D::setMap(const char* map)
{
  bool found = false;
  int mapIndex=0;
  for(unsigned int i=0; !found && i<maps.size(); i++){
    if(maps[i].mapName.compare(map)==0){
      mapIndex = i;
      found = true;
    }
  }
  if(found){
    currentMap = &maps[mapIndex];
  }else{
    char buf[4096];
    snprintf(buf, 4095, "Unknown map: \"%s\"",map);
    TerminalWarning(buf);
  }
}

void VectorLocalization2D::initialize(int _numParticles, const char* mapName, vector2f loc, float angle, float locationUncertainty, float angleUncertainty)
{
  static const bool debug = false;
  
  currentMap = &maps[0];
  bool found = false;
  for(unsigned int i=0; i<maps.size(); i++){
    if(maps[i].mapName.compare(mapName)==0){
      found = true;
      currentMap = &maps[i];
    }
  }
  if(!found){
    char buf[2048];
    snprintf(buf,2047,"Map %s not found in Atlas! Reverting to map %s.",mapName,maps[0].mapName.c_str());
    TerminalWarning(buf);
  }
  
  numParticles = _numParticles;
  particles.resize(numParticles);
  stage0Weights.resize(numParticles);
  stageRWeights.resize(numParticles);
  if(debug) printf(" Initializing particles: 0.0%%\r");
  fflush(stdout);
  for(int i=0; i<numParticles; i++){
    particles[i] = Particle2D(DBL_MAX,DBL_MAX,0.0,0.0);
  }
  for(int i=0; i<numParticles; i++){
    if(debug && (i%10)==0){
      printf(" Initializing particles: %5.1f%%\r",float(i)/float(numParticles)*100.0);
      fflush(stdout);
    }
    particles[i] = createParticle(currentMap, loc, angle, locationUncertainty, angleUncertainty);
  }
  
  computeLocation(loc, angle);
  
  laserEval.numCorrespondences = 0;
  laserEval.numObservedPoints = 0;
  laserEval.runTime = 0;
  laserEval.stage0Weights = 0;
  laserEval.stageRWeights = 0;
  
  pointCloudEval.numCorrespondences = 0;
  pointCloudEval.numObservedPoints = 0;
  pointCloudEval.runTime = 0;
  pointCloudEval.stage0Weights = 0;
  pointCloudEval.stageRWeights = 0;
  
  if(debug) printf("\nDone.\n");
}

void VectorLocalization2D::predict(float dx, float dy, float dtheta, const MotionModelParams &motionParams)
{
  lastDistanceMoved += vector2f(dx,dy).rotate(lastAngleTurned);
  lastAngleTurned += dtheta;
  for(int i=0; i<numParticles; i++){
    predictParticle(particles[i], dx, dy, dtheta, motionParams);
  }
}

float VectorLocalization2D::motionModelWeight(vector2f loc, float angle, const MotionModelParams &motionParams)
{
  float sqDensityKernelSize = sq(motionParams.kernelSize);
  float w = 0.0;
  float incr = 1.0/float(numParticles);
  for(int i=0; i<numParticles; i++){
    if( (loc-particles[i].loc).sqlength() < sqDensityKernelSize )
      w += incr;
  }
  return w;
}

float VectorLocalization2D::observationWeightPointCloud(vector2f loc, float angle, vector< vector2f >& pointCloud, vector< vector2f >& pointNormals, const PointCloudParams & pointCloudParams)
{
  //static const bool UseAnalyticRender = false;
  //static const bool debug = false;
  float logOutOfRangeProb = pointCloudParams.logOutOfRangeProb;
  float logObstacleProb = pointCloudParams.logObstacleProb;
  //float logShortHitProb = pointCloudParams.logShortHitProb;
  float corelationFactor = pointCloudParams.corelationFactor;
  float minRange = pointCloudParams.minRange;
  float maxRange = pointCloudParams.maxRange;
  float sqMaxRange = sq(maxRange);
  float stdDev = pointCloudParams.stdDev;
  //float curAngle;
  
  vector<line2f> lines;

  float a0 = angle-0.5*pointCloudParams.fieldOfView;
  float a1 = angle+0.5*pointCloudParams.fieldOfView;
  if(UseAnalyticRender){
    lineCorrespondences = currentMap->getRayToLineCorrespondences(loc, angle, a0, a1, pointCloud, minRange, maxRange, true, &lines);
  }else{
    lineCorrespondences = currentMap->getRayToLineCorrespondences(loc, angle, a0, a1, pointCloud, minRange, maxRange);
    lines = currentMap->lines;
  }
  
  float logTotalWeight = 0.0;
  
  Matrix2f rotMat1; 
  rotMat1 = Rotation2Df(angle);
  
  int numCorrespondences = 0;
  Vector2f heading, scanPoint, lineNorm, curPoint, attraction, locE(V2COMP(loc)), rotatedNormal;
  
  for(unsigned int i=0; i<pointCloud.size(); i++){
    curPoint = rotMat1*Vector2f(V2COMP(pointCloud[i])) + locE;
    attraction = Vector2f(0.0,0.0);
    if(lineCorrespondences[i]>=0){
      numCorrespondences++;
      if(UseAnalyticRender){
        attraction = observationFunction(lines[lineCorrespondences[i]], curPoint);
      }else{
        attraction = observationFunction(currentMap->lines[lineCorrespondences[i]], curPoint);
      }
      //logTotalWeight += -attraction.squaredNorm()*corelationFactor/stdDev;
      /**/
      if(UseAnalyticRender){
        lineNorm = Vector2f(V2COMP(lines[lineCorrespondences[i]].Perp()));
      }else{
        lineNorm = Vector2f(V2COMP(currentMap->lines[lineCorrespondences[i]].Perp()));
      }
      rotatedNormal = rotMat1*Vector2f(V2COMP(pointNormals[i]));
      if(true || fabs(lineNorm.dot(rotatedNormal)) > pointCloudParams.minCosAngleError){
        logTotalWeight += -attraction.squaredNorm()*corelationFactor/stdDev;
      }else{
        logTotalWeight += logObstacleProb*corelationFactor;
      }
      /**/
    }else if( pointCloud[i].sqlength()<sqMaxRange){
      logTotalWeight += logObstacleProb*corelationFactor;
    }else{
      logTotalWeight += logOutOfRangeProb*corelationFactor;
    }
  }
  //return numCorrespondences;
  return exp(logTotalWeight);
}

float VectorLocalization2D::observationWeightLidar(vector2f loc, float angle, const LidarParams &lidarParams, const vector<Vector2f> &laserPoints)
{
  //static const bool debug = false;
  float numRays = lidarParams.numRays;
  float minRange = lidarParams.minRange;
  float maxRange = lidarParams.maxRange;
  float logOutOfRangeProb = lidarParams.logOutOfRangeProb;
  float logObstacleProb = lidarParams.logObstacleProb;
  float logShortHitProb = lidarParams.logShortHitProb;
  
  vector2f laserLoc = loc + vector2f(lidarParams.laserToBaseTrans.x(),lidarParams.laserToBaseTrans.y()).rotate(angle);
  vector<line2f> lines;
  if(UseAnalyticRender){
    lineCorrespondences = currentMap->getRayToLineCorrespondences(laserLoc, angle, lidarParams.angleResolution, numRays, 0.0, maxRange, true, &lines);
  }else{
    lineCorrespondences = currentMap->getRayToLineCorrespondences(laserLoc, angle, lidarParams.angleResolution, numRays, 0.0, maxRange, false, NULL);
  }
  
  float *scanRays = lidarParams.laserScan;
  //float curAngle;
  Vector2f attraction;
  //float midScan = float(numRays)/2.0;
  
  Matrix2f robotAngle;
  robotAngle = Rotation2Df(angle);
  Vector2f heading, scanPoint, laserLocE(V2COMP(laserLoc));
  
  float logTotalWeight = 0.0;
  
  Vector2f scanPoint2, scanDir, lineDir;
  Matrix2f robotAngle2;
  robotAngle2 = Rotation2Df(angle+lidarParams.angleResolution);
  for(int i=0; i<numRays-1; i++){
    if(scanRays[i]>maxRange || scanRays[i]<minRange){		//si esta fuera de los rangos del laser, Lo identifica como obstaculo
      logTotalWeight += logObstacleProb*lidarParams.correlationFactor;
      continue;
    }
    if(lineCorrespondences[i]>=0){		//haya alguna linea
      scanPoint = laserLocE + robotAngle*laserPoints[i];
      scanPoint2 = laserLocE + robotAngle2*laserPoints[i+1];
      scanDir = (scanPoint2-scanPoint).normalized();
      if(UseAnalyticRender){
        lineDir = Vector2f(V2COMP(lines[lineCorrespondences[i]].Dir()));
      }else{
        lineDir = Vector2f(V2COMP(currentMap->lines[lineCorrespondences[i]].Dir()));
      }
      if(fabs(lineDir.dot(scanDir))>lidarParams.minCosAngleError){
        if(UseAnalyticRender)
          attraction = observationFunction(lines[lineCorrespondences[i]], scanPoint);
        else
          attraction = observationFunction(currentMap->lines[lineCorrespondences[i]], scanPoint);
        logTotalWeight += -min(attraction.squaredNorm()/lidarParams.lidarStdDev, -logShortHitProb)*lidarParams.correlationFactor;
      }else{
        logTotalWeight += logObstacleProb*lidarParams.correlationFactor;
      }
      
    }else if(scanRays[i]<maxRange){
      logTotalWeight += logObstacleProb*lidarParams.correlationFactor;
    }else{
      logTotalWeight += logOutOfRangeProb*lidarParams.correlationFactor;
    }
  }
  //return numCorrespondences;
  return exp(logTotalWeight);
}

void VectorLocalization2D::computeParticleWeights(vector2f deltaLoc, float deltaAngle, vector2f minLocStdDev, float minAngleStdDev, const MotionModelParams &motionParams)
{
  static const bool debug = false;
  
  int N = particles.size();
  vector<vector2f> locMean(2*N);
  vector<vector2f> locStdDev(2*N);
  vector<float> angleMean(2*N);
  vector<float> angleStdDev(2*N);
  
  currentLocStdDev.x = max(minLocStdDev.x,currentLocStdDev.x);
  currentLocStdDev.y = max(minLocStdDev.y,currentLocStdDev.y);
  currentAngleStdDev = max(minAngleStdDev,currentAngleStdDev);
  
  // Compute kernel supports for motion model weights
  float deltaTranslation = deltaLoc.length();
  
  for(int i=0; i<N; i++){
    angleMean[i] = particles[i].angle;
    angleStdDev[i] = 1.0/sq( max( float(fabs(deltaAngle))*motionParams.Alpha1 + deltaTranslation*motionParams.Alpha2 , currentAngleStdDev) );
    locMean[i] = particles[i].loc;
    locStdDev[i].x = 1.0/sq(max( (deltaLoc.rotate(particles[i].angle).x)*motionParams.Alpha3, currentLocStdDev.x));
    locStdDev[i].y = 1.0/sq(max( (deltaLoc.rotate(particles[i].angle).y)*motionParams.Alpha3, currentLocStdDev.y));
  }
  // Compute motion model weights from kernel supports
  for(int i=0; i<N; i++){
    Particle2D &p1 = particles[i];
    Particle2D &p2 = particlesRefined[i];
    p1.weight = 0.0;
    p2.weight = 0.0;
    for(int j=0; j<N; j++){
      p1.weight += exp(-sq(p1.loc.x - locMean[j].x)*locStdDev[j].x -sq(p1.loc.y - locMean[j].y)*locStdDev[j].y  -sq(angle_diff(p1.angle , angleMean[j]))*angleStdDev[j] );
      p2.weight += exp(-sq(p2.loc.x - locMean[j].x)*locStdDev[j].x -sq(p2.loc.y - locMean[j].y)*locStdDev[j].y  -sq(angle_diff(p2.angle , angleMean[j]))*angleStdDev[j] );
    }
  }
  
  if(debug){
    printf("Motion model weights:\n");
    for(int i=0; i<N; i++){
      printf("%2d unrefined:%20.16f, %20.16f refined:%20.16f, %20.16f\n",i, particles[i].weight,stage0Weights[i], particlesRefined[i].weight,stageRWeights[i]);
    }
  }
  
  // Apply observation function weights
  for(int i=0; i<N; i++){
    particles[i].weight *= stage0Weights[i];
    particlesRefined[i].weight *= stageRWeights[i];
  }
  
  // Compute kernel supports for sampling density function
  for(int i=0; i<N; i++){
    angleMean[i] = particles[i].angle;
    angleStdDev[i] = 1.0/sq( max( float(fabs(deltaAngle))*motionParams.Alpha1 + deltaTranslation*motionParams.Alpha2 , currentAngleStdDev) );
    locMean[i] = particles[i].loc;
    locStdDev[i].x = 1.0/sq(max( (deltaLoc.rotate(particles[i].angle).x)*motionParams.Alpha3, currentLocStdDev.x));
    locStdDev[i].y = 1.0/sq(max( (deltaLoc.rotate(particles[i].angle).y)*motionParams.Alpha3, currentLocStdDev.y));
    
    angleMean[N+i] = particlesRefined[i].angle;
    angleStdDev[N+i] = 1.0/sq( max( float(fabs(deltaAngle))*motionParams.Alpha1 + deltaTranslation*motionParams.Alpha2 , currentAngleStdDev) );
    locMean[N+i] = particlesRefined[i].loc;
    locStdDev[N+i].x = 1.0/sq(max( (deltaLoc.rotate(particlesRefined[i].angle).x)*motionParams.Alpha3, currentLocStdDev.x));
    locStdDev[N+i].y = 1.0/sq(max( (deltaLoc.rotate(particlesRefined[i].angle).y)*motionParams.Alpha3, currentLocStdDev.y));
  }
  // Apply sampling density function weights from kernel supports
  for(int i=0; i<N; i++){
    Particle2D &p1 = particles[i];
    Particle2D &p2 = particlesRefined[i];
    float w1 = 0.0;
    float w2 = 0.0;
    for(int j=0; j<2*N; j++){
      w1 += exp(-sq(p1.loc.x - locMean[j].x)*locStdDev[j].x -sq(p1.loc.y - locMean[j].y)*locStdDev[j].y  -sq(angle_diff(p1.angle , angleMean[j]))*angleStdDev[j] );
      w2 += exp(-sq(p2.loc.x - locMean[j].x)*locStdDev[j].x -sq(p2.loc.y - locMean[j].y)*locStdDev[j].y  -sq(angle_diff(p2.angle , angleMean[j]))*angleStdDev[j] );
    }
    p1.weight = p1.weight/w1;
    p2.weight = p2.weight/w2;
  }
  
  if(debug){
    printf("Final weights:\n");
    for(int i=0; i<N; i++){
      printf("%2d unrefined:%20.16f refined:%20.16f\n",i, particles[i].weight, particlesRefined[i].weight);
    }
  }
}

void VectorLocalization2D::updateLidar(const LidarParams &lidarParams, const MotionModelParams & motionParams)
{
  static const bool debug = false;
  
  double tStart = GetTimeSec();
  
  // Transform laserpoints to robot frame
  vector< Vector2f > laserPoints(lidarParams.numRays);
  for(int i=0; i<lidarParams.numRays; i++)
  {
    laserPoints[i] = lidarParams.scanHeadings[i]*lidarParams.laserScan[i];
    Vector2f aux(laserPoints[i].x(),laserPoints[i].y());
    laserPoints[i] = Vector2f(aux.y(),-aux.x()) + lidarParams.laserToBaseTrans;
  }
  //Compute the sampling density
  float sqDensityKernelSize = sq(lidarParams.kernelSize);
  float totalDensity = 0.0;
  unsigned int N = particlesRefined.size();
  if(samplingDensity.size()!=N)
    samplingDensity.resize(N);
  if(debug) printf("\nParticle samplingDensity:\n");
  for(unsigned int i=0; i<N; i++)
  {
    float w = 0.99;
    for(unsigned int j=0; j<N; j++)
    {
      if(i==j)
        continue;
      if( (particlesRefined[j].loc - particlesRefined[i].loc).sqlength() < sqDensityKernelSize && fabs(angle_diff(particlesRefined[j].angle, particlesRefined[i].angle))<RAD(20.0))
      //if( (particlesRefined[j].loc - particlesRefined[i].loc).sqlength() < sqDensityKernelSize)
        w++;
    }
    samplingDensity[i] = w;
    totalDensity += w;
    if(debug) printf("%2d:%f\n",i,w);
  }
  // Normalize densities, not really necessary since resampling does not require normalized weights
  for(unsigned int i=0; i<N; i++){
    samplingDensity[i] /= totalDensity;
  }
  
  //Compute importance weights = observation x motion / samplingDensity
  if(debug) printf("\nParticle weights:\n");
  for(unsigned int i=0; i<N; i++)
  {
    Particle2D &p = particlesRefined[i];
    float w1 = observationWeightLidar(p.loc, p.angle, lidarParams, laserPoints);
    float w2 = motionModelWeight(p.loc, p.angle, motionParams);
    p.weight = w1*w2/samplingDensity[i];
    if(debug) printf("%2d: %f , %f , %f\n",i,w1,w2,p.weight);
  }  
  updateTime = GetTimeSec() - tStart;
}

void VectorLocalization2D::updatePointCloud(vector< vector2f >& pointCloud, vector< vector2f >& pointNormals, const MotionModelParams &motionParams, const PointCloudParams &pointCloudParams)
 {
  static const bool debug = false;
  
  double tStart = GetTimeSec();
  
  //Compute the sampling density
  float sqDensityKernelSize = sq(pointCloudParams.kernelSize);
  float totalDensity = 0.0;
  unsigned int N = particlesRefined.size();
  if(samplingDensity.size()!=N)
    samplingDensity.resize(N);
  if(debug) printf("\nParticle samplingDensity:\n");
  for(unsigned int i=0; i<N; i++){
    float w = 0.99;
    for(unsigned int j=0; j<N; j++){
      if(i==j)
        continue;
      if( (particlesRefined[j].loc - particlesRefined[i].loc).sqlength() < sqDensityKernelSize && fabs(angle_diff(particlesRefined[j].angle, particlesRefined[i].angle))<RAD(20.0))
        //if( (particlesRefined[j].loc - particlesRefined[i].loc).sqlength() < sqDensityKernelSize)
        w++;
    }
    samplingDensity[i] = w;
    totalDensity += w;
    if(debug) printf("%2d:%f\n",i,w);
  }
  // Normalize densities, not really necessary since resampling does not require normalized weights
  for(unsigned int i=0; i<N; i++){
    samplingDensity[i] /= totalDensity;
  }
  
  //Compute importance weights = observation x motion / samplingDensity
  if(debug) printf("\nParticle weights:\n");
  for(unsigned int i=0; i<N; i++){
    Particle2D &p = particlesRefined[i];
    float w1 = observationWeightPointCloud(p.loc, p.angle, pointCloud, pointNormals, pointCloudParams);
    float w2 = motionModelWeight(p.loc, p.angle, motionParams);
    p.weight = w1*w2/samplingDensity[i];
    if(debug) printf("%2d: %f , %f , %f\n",i,w1,w2,p.weight);
  }
  
  updateTime = GetTimeSec() - tStart;
}

void VectorLocalization2D::predictParticle(Particle2D& p, float dx, float dy, float dtheta, const MotionModelParams &motionParams)
{
  static const bool debug = false;
  
  if(debug) printf("predict before: %7.2f,%7.2f %6.2f\u00b0 ",p.loc.x,p.loc.y,DEG(p.angle));
  //Motion model
  vector2f delta(dx,dy);
  float d_trans = delta.length();
  
  //Predict rotation
  dtheta += randn(motionParams.Alpha1*dtheta,0.0f)+randn(motionParams.Alpha2*d_trans,0.0f);
  p.angle = angle_mod(p.angle+dtheta);
  //Predict translation

  if(d_trans>FLT_MIN)
  {
    delta = delta.rotate(p.angle); 
    delta = delta.norm(d_trans + randn(motionParams.Alpha3*d_trans,0.0f));
  }
  if(debug) printf("delta: %f,%f %f\u00b0 ",V2COMP(delta),DEG(dtheta));
  p.loc += delta;
  if(debug) printf("after: %7.2f,%7.2f %6.2f\u00b0\n",p.loc.x,p.loc.y,DEG(p.angle));
}

inline Vector2f VectorLocalization2D::attractorFunction(line2f l, Vector2f p, float attractorRange, float margin)
{
  static const bool debug = false;
  
  Vector2f attraction(0.0,0.0), dir(V2COMP(l.Dir())), p0(V2COMP(l.P0())), p1(V2COMP(l.P1()));
  
  float location = (p-p0).dot(dir);
  
  if(location<-margin || location>l.Length()+margin){
    return attraction;
  }
  
  attraction = (p-p0) - dir*location ;
  
  float d = attraction.norm();
  /*
  if(d>0.5*attractorRange){
    float d2 = max(0.0f, attractorRange - d);
    attraction *= 2.0f*d2/attractorRange;
  }
  */
  if(d>attractorRange){
    attraction = Vector2f::Zero();
  }
  if(debug){
    debugLines.push_back( line2f( vector2f(p.x(),p.y()) ,vector2f((p-attraction).x(),(p-attraction).y()) ) );
  }
  return attraction;
}

inline Vector2f VectorLocalization2D::observationFunction(line2f l, Vector2f p)
{
  static const bool debug = false;
  Vector2f attraction(0.0,0.0), dir(V2COMP(l.Dir())), p0(V2COMP(l.P0())), p1(V2COMP(l.P1()));
  
  float location = (p-p0).dot(dir); // esto nos deja en location un valor absoluto proximo a 1 si el punto pertenece a la linea, a 0<alfa<1 si no
  attraction = (p-p0) - dir*location ;
  if(debug){
    debugLines.push_back( line2f( vector2f(p.x(),p.y()) ,vector2f((p-attraction).x(),(p-attraction).y()) ) );
    const float crossSize = 0.002;
    debugLines.push_back( line2f( vector2f(p.x()+crossSize,p.y()) , vector2f(p.x()-crossSize,p.y())) );
    debugLines.push_back( line2f( vector2f(p.x(),p.y()+crossSize) , vector2f(p.x(),p.y()-crossSize)) );
  }
  return attraction;
}

void VectorLocalization2D::getPointCloudGradient(vector2f loc, float angle, vector2f& locGrad, float& angleGrad, const std::vector< vector2f >& pointCloud, const std::vector< vector2f >& pointNormals, float& logWeight, const VectorLocalization2D::PointCloudParams& pointCloudParams, const vector<int> & lineCorrespondences, const vector<line2f> &lines)
{
  //static const bool UseAnalyticRender = false;
  static const bool debug = false;
  FunctionTimer *ft;
  if(EnableProfiling)
    ft = new FunctionTimer(__FUNCTION__);
  
  //float numTotalPoints = pointCloud.size();
  
  if(EnableProfiling) ft->Lap(__LINE__);
  //vector<int> lineCorrespondences;
  //vector<line2f> lines;
  
  /*
  float minRange = pointCloudParams.minRange;
  float maxRange = pointCloudParams.maxRange;
  float a0 = angle-0.5*pointCloudParams.fieldOfView;
  float a1 = angle+0.5*pointCloudParams.fieldOfView;
  if(UseAnalyticRender){
    lineCorrespondences = currentMap->getRayToLineCorrespondences(loc, angle, a0, a1, pointCloud, minRange, maxRange, true, &lines);
  }else{
    lineCorrespondences = currentMap->getRayToLineCorrespondences(loc, angle, a0, a1, pointCloud, minRange, maxRange);
    lines = currentMap->lines;
  }
  */
  if(EnableProfiling) ft->Lap(__LINE__);
  
  float numPoints = 0;
  
  float cosAngle;
  int noCorrespondences = 0;
  logWeight = 0.0;
  
  int numObservedPoints = int(pointCloud.size());
  pointCloudEval.numObservedPoints = numObservedPoints;
  
  //Construct gradients per point in point cloud
  gradients2.resize(numObservedPoints);
  points2.resize(numObservedPoints);
  int numPointsInt = 0;
  
  Vector2f curPoint, locE(V2COMP(loc)), attraction, lineNorm, rotatedNormal;
  Matrix2f rotMat1;
  rotMat1 = Rotation2Df(angle);
  
  for(int i=0; i<numObservedPoints; i++){
    curPoint = rotMat1*Vector2f(V2COMP(pointCloud[i])) + locE;
    rotatedNormal = rotMat1*Vector2f(V2COMP(pointNormals[i]));
    attraction = Vector2f(0,0);
    
    if(lineCorrespondences[i]>=0 && lineCorrespondences[i]<int(lines.size())){
      lineNorm = Vector2f(V2COMP(lines[lineCorrespondences[i]].Perp()));
      cosAngle = fabs(lineNorm.dot(rotatedNormal));
      
      //Attraction only for valid correspondences
      if(cosAngle > pointCloudParams.minCosAngleError){
        attraction = attractorFunction(lines[lineCorrespondences[i]], curPoint,pointCloudParams.attractorRange, pointCloudParams.correspondenceMargin);
        //Add the point and attraction (only if non-zero)
        //logWeight += -min(attraction.squaredNorm()/pointCloudParams.stdDev, -pointCloudParams.logShortHitProb)*pointCloudParams.corelationFactor;
        
        gradients2[numPointsInt] = attraction;
        points2[numPointsInt] = curPoint;
        numPointsInt++;
      }
    }else{
      noCorrespondences++;
    }
  }
  points2.resize(numPointsInt);
  gradients2.resize(numPointsInt);
  numPoints = float(numPointsInt);
  pointCloudEval.numCorrespondences = int(points2.size());
  
  if(debug) printf("No correspondences: %d/%d \n",noCorrespondences,int(pointCloud.size()));
  
  if(numPoints<pointCloudParams.minPoints){
    locGrad.zero();
    angleGrad = 0.0;
    return;
  }
  
  //Estimate translation and rotation
  Vector2f locGradE(0,0);
  Vector2f heading(0,0), curHeading, r;
  float headingAngle;
  pointCloudEval.meanSqError = 0.0;
  for(int i = 0; i<numPointsInt; i++){
    r = points2[i] - locE;
    pointCloudEval.meanSqError += gradients2[i].squaredNorm();
    if(r.squaredNorm()<sq(0.001))
      continue;
    
    locGradE += gradients2[i];
    headingAngle = eigenCross(r,gradients2[i]);
    curHeading = Vector2f(cos(headingAngle),sin(headingAngle));
    heading += curHeading;
  }
  locGradE = locGradE/numPoints;
  locGrad.set(locGradE.x(),locGradE.y());
  heading = heading/numPoints;
  pointCloudEval.meanSqError = pointCloudEval.meanSqError/numPoints;
  
  angleGrad = bound(atan2(heading.y(),heading.x()),-pointCloudParams.maxAngleGradient,pointCloudParams.maxAngleGradient);
  
  locGrad = locGrad.bound(pointCloudParams.maxLocGradient);
  if(debug) printf("LocGrad: %6.2f %6.2f AngleGrad:%6.1f\u00b0\n",V2COMP(locGrad), DEG(angleGrad));
    
  if(EnableProfiling) delete ft;
}

void VectorLocalization2D::getLidarGradient(vector2f loc, float angle, vector2f& locGrad, float& angleGrad, float& logWeight, VectorLocalization2D::LidarParams lidarParams, const vector<Vector2f>& laserPoints, const vector<int> & lineCorrespondences, const vector<line2f> &lines)
{
  static const bool EnableProfiling = false;
  
  FunctionTimer *ft;
  if(EnableProfiling)
    ft = new FunctionTimer(__PRETTY_FUNCTION__);
  
  float logShortHitProb = lidarParams.logShortHitProb;
  Matrix2f robotAngle;
  robotAngle = Rotation2Df(angle);
  Vector2f laserLocE = Vector2f(V2COMP(loc)) + robotAngle*(lidarParams.laserToBaseTrans);
  vector2f laserLoc(laserLocE.x(), laserLocE.y());
  
  if(EnableProfiling) ft->Lap(__LINE__);
  
  /*
  if(UseAnalyticRender){
    lineCorrespondences = currentMap->getRayToLineCorrespondences(laserLoc, angle, lidarParams.angleResolution, lidarParams.numRays, lidarParams.minRange, lidarParams.maxRange, true, &lines);
  }else{
    lineCorrespondences = currentMap->getRayToLineCorrespondences(laserLoc, angle, lidarParams.angleResolution, lidarParams.numRays, lidarParams.minRange, lidarParams.maxRange);
  }
  */
  if(EnableProfiling) ft->Lap(__LINE__);
  
  points.clear();
  gradients.clear();
  static const bool debug = false;
  float *scanRays = lidarParams.laserScan;
  float minRange = lidarParams.minRange;
  float maxRange = lidarParams.maxRange;
  float curAngle;
  Vector2f heading, heading2;
  float numPoints = 0;
  float midScan = float(lidarParams.numRays)/2.0;
  float cosAngle;
  int noCorrespondences = 0;
  logWeight = 0.0;
  curAngle = angle - midScan*lidarParams.angleResolution;
  //Construct gradients per point in point cloud
  if(EnableProfiling) ft->Lap(__LINE__);
  
  Matrix2f robotAngle2;
  robotAngle2 = Rotation2Df(angle+lidarParams.angleResolution);
  Vector2f scanPoint, scanPoint2, scanDir, lineDir, attraction;
  
  //TODO: Speed up this section!!
  //===========================================================================
  laserEval.numObservedPoints = 0;
  for(int i=0; i<lidarParams.numRays-1; i++, curAngle+=lidarParams.angleResolution){
    if(scanRays[i]<minRange || scanRays[i]>maxRange)
      continue;
    laserEval.numObservedPoints++;  
    scanPoint = laserLocE + robotAngle*laserPoints[i];
    
    if(lineCorrespondences[i]>=0){
      scanPoint2 = laserLocE + robotAngle2*laserPoints[i+1];
      scanDir = (scanPoint2-scanPoint).normalized();
      if(UseAnalyticRender){
        lineDir = Vector2f(V2COMP(lines[lineCorrespondences[i]].Dir()));
        attraction = attractorFunction(lines[lineCorrespondences[i]], scanPoint, lidarParams.attractorRange, lidarParams.correspondenceMargin);
      }else{
        lineDir = Vector2f(V2COMP(currentMap->lines[lineCorrespondences[i]].Dir()));
        attraction = attractorFunction(currentMap->lines[lineCorrespondences[i]], scanPoint, lidarParams.attractorRange, lidarParams.correspondenceMargin);
      }
      logWeight += -min(attraction.squaredNorm()/lidarParams.lidarStdDev, -logShortHitProb)*lidarParams.correlationFactor;
      cosAngle = fabs(lineDir.dot(scanDir));
      if(cosAngle>lidarParams.minCosAngleError){
        gradients.push_back(attraction);
        points.push_back(scanPoint);
      }
    }else{
      noCorrespondences++;
    }
    
  }  
  numPoints = float(points.size());
  laserEval.numCorrespondences = int(points.size());
  
  //if(debug) printf("No correspondences: %d/%d numPoints:%d\n",noCorrespondences,lidarParams.numRays,int(numPoints));
  //===========================================================================
  
  if(EnableProfiling) ft->Lap(__LINE__);
  
  if(numPoints<lidarParams.minPoints){
    locGrad.zero();
    angleGrad = 0.0;
    if(EnableProfiling) delete ft;
    return;
  }
  
  //Estimate translation and rotation
  locGrad.zero();
  heading = Vector2f(0,0);
  float headingAngle;
  laserEval.meanSqError = 0.0;
  Vector2f locE(V2COMP(loc)), r, locGradE(0.0,0.0);
  for(unsigned int i = 0; i<gradients.size(); i++){
    r = points[i]-locE;
    laserEval.meanSqError += gradients[i].squaredNorm();
    if(r.squaredNorm()<sq(0.03))
      continue;
    locGradE += gradients[i];
    headingAngle = eigenCross(r, gradients[i]);
    heading += Vector2f(cos(headingAngle),sin(headingAngle));
  }
  locGradE /= numPoints;
  locGrad.set(locGradE.x(),locGradE.y());
  heading /= numPoints;
  laserEval.meanSqError /= numPoints;
  
  if(EnableProfiling) ft->Lap(__LINE__);
    
  locGrad = locGrad.bound(lidarParams.maxLocGradient);
  angleGrad = bound(atan2(heading.y(),heading.x()),-lidarParams.maxAngleGradient,lidarParams.maxAngleGradient);
  if(EnableProfiling) delete ft;
  if(debug) printf("Gradient: %.4f,%.4f %.2f\u00b0\n",V2COMP(locGrad),DEG(angleGrad));
}
//refine del paper
void VectorLocalization2D::refineLocationLidar(vector2f& loc, float& angle, float& initialWeight, float& finalWeight, const LidarParams &lidarParams, const vector<Vector2f> &laserPoints)
{
  static const bool debug = false;
  
  //Do gradient descent for this particle
  vector2f locGrad(0.0,0.0);
  float angleGrad = 0.0;
  bool beingRefined = true;
  float weight;
  
  if(debug) printf("before: %.4f,%.4f %.2f\u00b0\n",V2COMP(loc),DEG(angle));
  Matrix2f robotAngle;
  robotAngle = Rotation2Df(angle);  
  Vector2f laserLocE = Vector2f(V2COMP(loc)) + robotAngle*(lidarParams.laserToBaseTrans);
  vector2f laserLoc(laserLocE.x(), laserLocE.y());
  vector<line2f> lines;
  
  if(UseAnalyticRender){
    lineCorrespondences = currentMap->getRayToLineCorrespondences(laserLoc, angle, lidarParams.angleResolution, lidarParams.numRays, lidarParams.minRange, lidarParams.maxRange, true, &lines);
  }else{
    lineCorrespondences = currentMap->getRayToLineCorrespondences(laserLoc, angle, lidarParams.angleResolution, lidarParams.numRays, lidarParams.minRange, lidarParams.maxRange);
  }
  
  for(int i=0; beingRefined && i<lidarParams.numSteps; i++)
  {
    getLidarGradient(loc,angle,locGrad,angleGrad,weight,lidarParams, laserPoints, lineCorrespondences, lines);
    if(i==0) initialWeight = exp(weight);
    loc -= lidarParams.etaLoc*locGrad;
    angle -= lidarParams.etaAngle*angleGrad;
    beingRefined = fabs(angleGrad)>lidarParams.minRefineFraction*lidarParams.maxAngleGradient && locGrad.sqlength()>sq(lidarParams.minRefineFraction*lidarParams.maxLocGradient);
  }
  
  if(debug) printf("after: %.4f,%.4f %.2f\u00b0\n",V2COMP(loc),DEG(angle));
  finalWeight = exp(weight);
}



bool VectorLocalization2D::inLine(int numPoint, const std::vector< Vector2f >& pointsLaser)
{
	float xT = 0.0, yT = 0.0, xyT = 0.0, xxT = 0.0, yyT = 0.0;
	int limit = 3;
	for (int i = numPoint - limit; i <= numPoint + limit; i++)
	{
		xT += pointsLaser[i].x();
		yT += pointsLaser[i].y();
		xyT += pointsLaser[i].x() * pointsLaser[i].y();
		xxT += pointsLaser[i].x() * pointsLaser[i].x();
		yyT += pointsLaser[i].y() * pointsLaser[i].y();
	}

	int numRay = 11.0;
	float xAV = xT / numRay;
	float yAV = yT / numRay;
	float cov = xyT / numRay;
	float xVar = xxT / numRay - xAV * xAV;	
	float yVar = yyT / numRay - yAV * yAV;
	float a = cov / xVar;
	float b = yAV - a * xAV;
	float a2 = cov / yVar;	
	float b2 = xAV - a2 * yAV;
// 	float m = (xyT - (xT * yT) / 10.0) / (xxT - (xT*xT) / 10.0);
// 	float b = yAV - m * xAV

//	LINE y = ax + b + error
// //	error = y -ax -b	
// 	float Vres = eT /
	
	float eT = 0.0,eT2 = 0.0;
	for (int i = numPoint - limit; i < numPoint + limit; i++)
	{
		eT += std::abs(pointsLaser[i].y() - a * pointsLaser[i].x() - b);
		eT2 += std::abs(pointsLaser[i].y() - a2 * pointsLaser[i].x() - b2);
// // 		printf("error %f",pointsLaser[i].y() - a * pointsLaser[i].x() - b);
	}
	printf("error Total de %i, es %f \n",numPoint, eT2);
 	return eT2 < 80.0;
	return true;
}






void VectorLocalization2D::refineLocationPointCloud(vector2f& loc, float& angle, float& initialWeight, float& finalWeight, const std::vector< vector2f >& pointCloud, const std::vector< vector2f >& pointNormals, const VectorLocalization2D::PointCloudParams& pointCloudParams)
{
  static const bool debug = false;
  //FunctionTimer ft(__PRETTY_FUNCTION__);
  
  //Do gradient descent for this particle
  vector2f locGrad(0.0,0.0);
  float angleGrad = 0.0;
  bool beingRefined = true;
  
  
  float a0 = angle-0.5*pointCloudParams.fieldOfView;
  float a1 = angle+0.5*pointCloudParams.fieldOfView;
  vector<line2f> lines;
  if(UseAnalyticRender){
    lineCorrespondences = currentMap->getRayToLineCorrespondences(loc, angle, a0, a1, pointCloud, pointCloudParams.minRange, pointCloudParams.maxRange, true, &lines);
  }else{
    lineCorrespondences = currentMap->getRayToLineCorrespondences(loc, angle, a0, a1, pointCloud, pointCloudParams.minRange, pointCloudParams.maxRange);
    lines = currentMap->lines;
  }
  
  vector2f locPrev = loc;
  float anglePrev = angle;
  float weight;
  locCorrectionP0 = loc;
  for(int i=0; beingRefined && i<pointCloudParams.numSteps; i++){
    getPointCloudGradient(loc,angle,locGrad,angleGrad,pointCloud,pointNormals,weight, pointCloudParams, lineCorrespondences, lines);
    if(i==0) initialWeight = exp(weight);
    loc -= pointCloudParams.etaLoc*locGrad;
    angle -= pointCloudParams.etaAngle*angleGrad;
    beingRefined = fabs(angleGrad)>pointCloudParams.minRefineFraction*pointCloudParams.maxAngleGradient && locGrad.sqlength()>sq(pointCloudParams.minRefineFraction*pointCloudParams.maxLocGradient);
  }
  locCorrectionP1 = loc;
  finalWeight = exp(weight);
  if(debug) printf("gradient: %7.3f,%7.3f %6.1f\u00b0\n",V2COMP(loc-locPrev),DEG(angle-anglePrev));
}

void VectorLocalization2D::refineLidar(const LidarParams &lidarParams)
{
  //FunctionTimer ft(__PRETTY_FUNCTION__);
  double tStart = GetTimeSec();
  laserEval.stage0Weights = 0.0;
  laserEval.stageRWeights = 0.0;
  laserEval.lastRunTime = GetTimeSec();
  
  // Transform laserpoints to robot frame
  vector< Vector2f > laserPoints(lidarParams.numRays);
  for(int i=0; i<lidarParams.numRays; i++){
//     laserPoints[i] = lidarParams.laserToBaseTrans + lidarParams.laserToBaseRot*lidarParams.scanHeadings[i]*lidarParams.laserScan[i];
    laserPoints[i] = lidarParams.scanHeadings[i]*lidarParams.laserScan[i];
    Vector2f aux(laserPoints[i].x(),laserPoints[i].y());
    laserPoints[i] = Vector2f(aux.y(),-aux.x()) + lidarParams.laserToBaseTrans;
  }
  
//   list<int> listPointsFilter;
//   for(int i=0; i<lidarParams.numRays; i++)
//   {
//  	  if (!inLine(i,laserPoints))
// 	  {
		  // not in line
// 		  listPointsFilter.push_back(i);
// 		 printf("probar que tiene efecto!!\n");
// 	  }
//   }
  
//   while( !listPointsFilter.empty() )
//   {
// 	  laserPoints[listPointsFilter.front()] = Vector2f(10000,10000);
// 	  listPointsFilter.pop_front();
//   }
/*  
  for(int i=0; i<lidarParams.numRays; i++)
  {
	printf("punto laser %d --> (%f , %f)\n",i,laserPoints[i].x(),laserPoints[i].y());
  }*/
//   qFatal("fin");
//     printf(" -------------------------------- \n");
  
  
  
  
  
  particlesRefined = particles;
  if(lidarParams.numSteps>0){  
    for(int i=0; i<numParticles; i++){
      refineLocationLidar(particlesRefined[i].loc, particlesRefined[i].angle, stage0Weights[i], stageRWeights[i], lidarParams, laserPoints);
      laserEval.stage0Weights += stage0Weights[i];
      laserEval.stageRWeights += stageRWeights[i];
    }
  }
  refineTime = GetTimeSec() - tStart;
  laserEval.runTime = refineTime;
}


void VectorLocalization2D::reducePointCloud(const std::vector< vector2f >& pointCloud, const std::vector< vector2f >& pointNormals, vector< vector2f >& reducedPointCloud, vector< vector2f >& reducedPointNormals)
{
  static const bool debug = false;
  static const float eps = -RAD(0.001);
  
  vector<pair<float, int> > angles;
  size_t N = pointCloud.size();
  pair<float,int> valuePair;
  
  for(unsigned int i=0; i<N; i++){
    valuePair.first = pointCloud[i].angle();
    valuePair.second = i;
    angles.push_back(valuePair);
  }
  //N = min(angles.size(),N);
  sort<vector<pair<float, int> >::iterator >(angles.begin(), angles.end());
  
  
  reducedPointCloud.resize(N);
  reducedPointNormals.resize(N);
  int j=0;
  float lastAngle;
  for(unsigned int i=0; i<N; ){
    reducedPointCloud[j] = pointCloud[angles[i].second];
    reducedPointNormals[j] = pointNormals[angles[i].second];
    j++;
    lastAngle = angles[i].first;
    
    for(i++;i<N && angles[i].first<=lastAngle+eps;){
      i++;
    }
  }
  N = j;
  reducedPointCloud.resize(N);
  reducedPointNormals.resize(N);
  
  
  if(debug){
    printf("\n");
    for(unsigned int i=0; i<N; i++){
      //printf("%4d: %6.2f\u00b0 %4d\n",int(i),DEG((*it).first),(*it).second);
      printf("%4d: %6.2f\u00b0\n",int(i),DEG(angles[i].first));
    }
  }
}

void VectorLocalization2D::refinePointCloud(const vector<vector2f> &pointCloud, const vector<vector2f> &pointNormals, const PointCloudParams &pointCloudParams)
{
  //FunctionTimer ft(__PRETTY_FUNCTION__);
  double tStart = GetTimeSec();
  pointCloudEval.stage0Weights = 0.0;
  pointCloudEval.stageRWeights = 0.0;
  pointCloudEval.lastRunTime = GetTimeSec();
  
  /*
  vector< vector2f > reducedPointCloud, reducedPointNormals;
  reducePointCloud(pointCloud, pointNormals, reducedPointCloud, reducedPointNormals);
  */
  
  particlesRefined = particles;
  if(pointCloudParams.numSteps>0){
    for(int i=0; i<numParticles; i++){
      //refineLocationPointCloud(particlesRefined[i].loc, particlesRefined[i].angle, stage0Weights[i], stageRWeights[i],reducedPointCloud, reducedPointNormals, pointCloudParams);
      refineLocationPointCloud(particlesRefined[i].loc, particlesRefined[i].angle, stage0Weights[i], stageRWeights[i],pointCloud, pointNormals, pointCloudParams);
      pointCloudEval.stage0Weights += stage0Weights[i];
      pointCloudEval.stageRWeights += stageRWeights[i];
    }
  }
  refineTime = GetTimeSec() - tStart;
  pointCloudEval.runTime = refineTime;
}

void VectorLocalization2D::computeLocation(vector2f& loc, float& angle)
{
  vector2f heading;
  vector2f avgHeading;
  currentLocation.zero();
  avgHeading.zero();
  for(int i=0; i<numParticles; i++){
    currentLocation = currentLocation + particles[i].loc;
    heading.heading(particles[i].angle);
    avgHeading = avgHeading + heading;
  }
  currentAngle = avgHeading.angle();
  currentLocation = currentLocation/float(numParticles);
  
  currentLocStdDev.zero();
  currentAngleStdDev = 0.0;
  float xStdDev=0.0, yStdDev=0.0;
  for(int i=0; i<numParticles; i++){
    /*
    xStdDev += sq(float(particles[i].loc.x-currentLocation.x));
    yStdDev += sq(float(particles[i].loc.y-currentLocation.y));
    currentAngleStdDev += sq(float(angle_diff(particles[i].angle,currentAngle)));
    */
    xStdDev = max(xStdDev, float(fabs(particles[i].loc.x-currentLocation.x)));
    yStdDev = max(yStdDev, float(fabs(particles[i].loc.y-currentLocation.y)));
    currentAngleStdDev = max(currentAngleStdDev, float(fabs(angle_diff(particles[i].angle,currentAngle))));
  }
  /*
  currentAngleStdDev = sqrt(currentAngleStdDev/float(numParticles-1.0));
  xStdDev = sqrt(xStdDev/float(numParticles));
  yStdDev = sqrt(yStdDev/float(numParticles));
  */
  currentLocStdDev.set(xStdDev,yStdDev);
  loc = currentLocation;
  angle = currentAngle;
}

void VectorLocalization2D::resample(Resample type)
{
  switch(type){
    case NaiveResampling:{
      naiveResample();
    }break;
    case LowVarianceResampling:{
      lowVarianceResample();
    }break;
    case SensorResettingResampling:{
    }break;
  }
}

void VectorLocalization2D::lowVarianceResample()
{ 
  vector<Particle2D> newParticles;
  newParticles.resize(numParticles);
  float totalWeight = 0.0;
  float newWeight = 1.0/float(numParticles);
  int numRefinedParticles = (int) particlesRefined.size();
  
  refinedImportanceWeights = unrefinedImportanceWeights = 0.0;
  for(int i=0; i<numRefinedParticles; i++){
    //Get rid of particles with undefined weights
    if(isnan(particlesRefined[i].weight) || isinf(particlesRefined[i].weight) || particlesRefined[i].weight<0.0)
      particlesRefined[i].weight = 0.0;
    totalWeight += particlesRefined[i].weight;
    if(i<numParticles)
      refinedImportanceWeights += particlesRefined[i].weight;
    else
      unrefinedImportanceWeights += particlesRefined[i].weight;
  }
  
  if(totalWeight<FLT_MIN){
    //TerminalWarning("Particles have zero total weight!");
    for(int i=0; i<numParticles; i++){
      particles[i].weight = newWeight;
    }
    return;
    //exit(0);
  }
  
  float weightIncrement = totalWeight/float(numParticles);
  if(weightIncrement<FLT_MIN) TerminalWarning("Particle weights less than float precision");
  
  numRefinedParticlesSampled = numUnrefinedParticlesSampled = 0;
  float x = frand(0.0f,totalWeight);
  int j=0;
  float f=particlesRefined[0].weight;
  for(int i=0; i<numParticles; i++){
    while(f<x){
      j = (j+1)%numRefinedParticles;
      f += particlesRefined[j].weight;
    }
    if(j<numParticles)
      numRefinedParticlesSampled++;
    else
      numUnrefinedParticlesSampled++;
    
    newParticles[i] = particlesRefined[j];
    newParticles[i].weight = newWeight;
    // Si comentamos esto, todas las particulas se desplazan con ruido
     if(particlesRefined[i].weight < FLT_MIN)
    {
      //This particle was depleted: add replacement noise
      vector2f deltaLoc = vector2f(frand(-1.0,1.0),frand(-1.0,1.0))*0.05;
      float deltaAngle = frand(-1.0,1.0)*RAD(5.0);
      newParticles[i].loc += deltaLoc;
      newParticles[i].lastLoc += deltaLoc;
      newParticles[i].angle += deltaAngle;
    }
    x += weightIncrement;
  }
  
  particles = newParticles;
}

void VectorLocalization2D::naiveResample()
{
  vector<Particle2D> newParticles;
  static vector<Particle2D> oldParticles;
  newParticles.resize(numParticles);
  float totalWeight = 0.0;
  float newWeight = 1.0/numParticles;
  int numRefinedParticles = (int) particlesRefined.size();
  
  refinedImportanceWeights = unrefinedImportanceWeights = 0.0;
  int numInfs=0, numNans=0, numNegs=0;
  for(int i=0; i<numRefinedParticles; i++){
    if(isnan(particlesRefined[i].weight))
      numNans++;
    else if(isinf(particlesRefined[i].weight))
      numInfs++;
    else if(particlesRefined[i].weight<0.0)
      numNegs++;
    //Get rid of particles with undefined weights
    if(isnan(particlesRefined[i].weight) || isinf(particlesRefined[i].weight) || particlesRefined[i].weight<0.0)
      particlesRefined[i].weight = 0.0;
    totalWeight += particlesRefined[i].weight;
    if(i<numParticles)
      refinedImportanceWeights += particlesRefined[i].weight;
    else
      unrefinedImportanceWeights += particlesRefined[i].weight;
  }
  if(totalWeight<FLT_MIN){
    TerminalWarning("Particles have zero total weight!");
    printf("inf:%d nan:%d neg:%d\n",numInfs, numNans, numNegs);
    particles = oldParticles;
    return;
    //exit(0);
  }
  
  numRefinedParticlesSampled = numUnrefinedParticlesSampled = 0;
  for(int i=0; i<numParticles; i++){
    float x = frand(0.0f,totalWeight);
    float f=particlesRefined[0].weight;
    int j=0;
    while(f<x && j<numRefinedParticles-1){
      j++;
      f += particlesRefined[j].weight;
    }
    if(j<numParticles)
      numRefinedParticlesSampled++;
    else
      numUnrefinedParticlesSampled++;
    newParticles[i] = particlesRefined[j];
    newParticles[i].weight = newWeight;
  }
  particles = newParticles;
  oldParticles = particles;
}

void VectorLocalization2D::saveProfilingStats(FILE* f)
{
  fprintf(f, "%f, %f, ",refineTime, updateTime);
}

void VectorLocalization2D::saveRunLog(FILE* f)
{
  static const bool saveParticles = false;
  fprintf(f, "%d, %d, %.15f, %.15f, ",numRefinedParticlesSampled, numUnrefinedParticlesSampled, refinedImportanceWeights, unrefinedImportanceWeights);
  if(saveParticles){
    for(unsigned int i=0; i<particles.size(); i++){
      fprintf(f, "%.3f, %.3f, %.2f, %f, ",V2COMP(particles[i].loc), DEG(particles[i].angle), particles[i].weight);
    }
  }
}

void VectorLocalization2D::getEvalValues(VectorLocalization2D::EvalValues& _laserEval, VectorLocalization2D::EvalValues& _pointCloudEval)
{
  laserEval.stage0Weights /= float(numParticles);
  laserEval.stageRWeights /= float(numParticles);
  pointCloudEval.stage0Weights /= float(numParticles);
  pointCloudEval.stageRWeights /= float(numParticles);
  _laserEval = laserEval;
  _pointCloudEval = pointCloudEval;
}

void VectorLocalization2D::getUncertainty(float& _angleUnc, float& _locUnc)
{
  _angleUnc = currentAngleStdDev;
  _locUnc = currentLocStdDev.length();
}
