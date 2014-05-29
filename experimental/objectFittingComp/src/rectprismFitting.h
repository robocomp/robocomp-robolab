#ifndef RECTPRISMFITTING_H
#define RECTPRISMFITTING_H
#include "rectprismCloudParticle.h"

class RectPrismFitting
{
   
public:
  RectPrismFitting(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cl);
  ~RectPrismFitting();
  void sig_term();
  void run(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_, QVec &translation, QVec &rotation,QVec &width );
  //Vector V(const double& r);

  inline double getRandom() { return (rand()%32000)/32000.0; }
  inline bool isComputing () { return computing; }
  
private:
  
  bool computing;
  RectPrismCloudPFInputData input;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cup;
  RCParticleFilter<RectPrismCloudPFInputData, int, RectPrismCloudParticle, RCParticleFilter_Config> *pf;
  
  RCParticleFilter_Config c;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr readPCLCloud(QString name);
};

#endif