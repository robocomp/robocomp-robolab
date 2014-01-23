#ifndef RECTPRISMFITTING_H
#define RECTPRISMFITTING_H
#include "rect_prism_cloud_particle.h"
#include "fitting.h"

typedef pcl::PointXYZRGBA PointT;

class PfRectPrismFitting: public fitting
{
	// Define callback signature typedefs
	typedef void (sig_cb_fitting_addapt) (RectPrism);

	boost::thread captured_thread;
	mutable boost::mutex capture_mutex;
	boost::signals2::signal<sig_cb_fitting_addapt>* fitting_signal;

public:
	PfRectPrismFitting( int numparticles, pcl::PointCloud<PointT>::Ptr cloudToFit);
	~PfRectPrismFitting();
	void sig_term();
	void captureThreadFunction();
	void setPointCloud(  pcl::PointCloud<PointT>::Ptr cloud) { cloud2Fit=*cloud; }
	float getRandom(float var);
	inline double getRandom() { return (double(rand())/RAND_MAX); }
	inline RectPrism getBestFit() { return bestParticle.getRectPrism(); }
	inline RectPrismCloudParticle getBestParticle() { return bestParticle; }

private:
	RectPrismCloudPFInputData input;
	pcl::PointCloud<PointT> cloud2Fit;
	RCParticleFilter<RectPrismCloudPFInputData, int, RectPrismCloudParticle, RCParticleFilter_Config> *pf;

	RCParticleFilter_Config c;
	pcl::PointCloud<PointT>::Ptr cloud;
	RectPrismCloudParticle bestParticle;
};

#endif