#ifndef RECTPRISMCLOUDPARTICLE_H
#define RECTPRISMCLOUDPARTICLE_H

#include <iostream>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <cmath>

#include <particleFiltering/particleFilter.h>
#include <limits>
#include "shapes/rectprism.h"

#include <qmat/QMatAll>

class RectPrismCloudPFInputData
{
public:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target;
};

class RectPrismCloudParticle : public RCParticleFilter_Particle<RectPrismCloudPFInputData, int, RCParticleFilter_Config>
{
public:

  RectPrismCloudParticle();
  ~RectPrismCloudParticle();
  void initialize(const RectPrismCloudPFInputData &data, const int &control, const RCParticleFilter_Config *config);
  void adapt(const int &controlBack, const int &controlNew, const bool noValidCandidates);
  void computeWeight(const RectPrismCloudPFInputData &data);
  void initializeFromEigenValues(const RectPrismCloudPFInputData &data);
  void gypsyInitization(const RectPrismCloudPFInputData &data);
  QVec getTranslation();
  QVec getRotation();
  QVec getScale();
  float getRandom(float var);
  void setRectPrism (RectPrism r );
  void print(std::string v);

  void estimateEigenAndCentroid(const RectPrismCloudPFInputData &data, Eigen::Vector3f &eig_values, Eigen::Matrix3f &eig_vectors, Eigen::Vector4f &centroid);

  
  inline RectPrism getRectPrism() { return r; };

private:
  RectPrism r;
  
  //Random variance variables
  QVec varianceC;
  QVec varianceW;
  QVec varianceR;

};

#endif
