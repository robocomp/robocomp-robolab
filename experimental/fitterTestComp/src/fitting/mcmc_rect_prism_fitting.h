#ifndef OROL_MCMC_RECT_PRISM_FITTING
#define OROL_MCMC_RECT_PRISM_FITTING

#include "../shapes/rectprism.h"
#include "fitting.h"

#include <boost/thread.hpp>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>

/** \brief Marcov Chain Montecarlo fitting for rectangular prism
  * \author Marco A. Gutierrez <marcog@unex.es>
  * \ingroup fitting
  */

typedef pcl::PointXYZRGBA PointT;

class mcmcRectangularPrismFitting: public fitting
{
  // Define callback signature typedefs
  typedef void (sig_cb_fitting_addapt) (const boost::shared_ptr<RectPrism>&);

  pcl::PointCloud<PointT>::Ptr pointCloud2Fit;
  boost::shared_ptr<RectPrism> shape2Fit;
  boost::shared_ptr<RectPrism> bestFit;
  float weight;
  float bestweight;
  boost::thread captured_thread;
  mutable boost::mutex capture_mutex;
  boost::signals2::signal<sig_cb_fitting_addapt>* fitting_signal;
  float MAX_WIDTH;

  //Random variance variables
  QVec varianceC;
  QVec varianceS;
  QVec varianceR;
  
public:
  mcmcRectangularPrismFitting ( pcl::PointCloud<PointT>::Ptr cloud, QVec C, QVec S, QVec R );
  //Get and set cloud
  inline void setCloud (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) { cout<<"setcloud"<<endl; pointCloud2Fit=cloud; }
  inline pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getCloud () { return pointCloud2Fit; }
  //get and set rectangular prism
  inline void setRectangularPrism (boost::shared_ptr<RectPrism> shape) { shape2Fit=shape; }
  
  inline boost::shared_ptr<RectPrism> getRectangularPrism () { return shape2Fit; }
  inline boost::shared_ptr<RectPrism> getBest () { return bestFit; }
  
  inline void setVarianceCenter (QVec varianceC) { this->varianceC=varianceC; }
  inline QVec getVarianceCenter () { return varianceC; }
  
  inline void setVarianceScale (QVec varianceS) { this->varianceS=varianceS; }
  inline QVec getVarianceSize () { return varianceS; }
  
  inline void setVarianceRotation (QVec varianceR) { this->varianceR=varianceR; }
  inline QVec getVarianceRotation () { return varianceR; }
  
  //calcualte weight of the cloud and rectangular prism fitting
  float computeWeight();
  //do a step of the mcmc_fitting
  void adapt();
  
protected:
  void initRectangularPrism ();
  void captureThreadFunction ();
  void MarkovChainStepOnAll();
  void MarkovChainStepOnOne();
  float getRandom(float var);
  
};

#endif