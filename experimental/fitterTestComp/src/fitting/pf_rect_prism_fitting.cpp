#include "pf_rect_prism_fitting.h"

float PfRectPrismFitting::getRandom(float var)
{
  double U = double(rand())/RAND_MAX;
  double V = double(rand())/RAND_MAX;
  return sqrt(-2*log(U))*cos(2.*M_PIl*V)*var;
}

/**
  * \brief Default constructor
  */
// PfRectPrismFitting::PfRectPrismFitting()
// {
//   
//   c.particles=30 ;
//   
//   captured_thread = boost::thread (&PfRectPrismFitting::captureThreadFunction, this);
//   fitting_signal = createSignal<sig_cb_fitting_addapt> ();
//   
//   //input.cloud_target=*cloud2Fit; 
//   
//   pf = new RCParticleFilter<RectPrismCloudPFInputData, int, RectPrismCloudParticle, RCParticleFilter_Config> (&c, input, 0);
// }

/**
  * \brief Default constructor
  */
PfRectPrismFitting::PfRectPrismFitting( int numparticles,  pcl::PointCloud<PointT>::Ptr cloudToFit)
{
	c.particles=numparticles;

	captured_thread = boost::thread (&PfRectPrismFitting::captureThreadFunction, this);
	fitting_signal = createSignal<sig_cb_fitting_addapt> ();

	this->cloud2Fit = *cloudToFit;

	input.cloud_target=cloud2Fit;
	pf = new RCParticleFilter<RectPrismCloudPFInputData, int, RectPrismCloudParticle, RCParticleFilter_Config> (&c, input, 0);
}

/**
  * \brief Default destructor
  */
PfRectPrismFitting::~PfRectPrismFitting()
{

}

void PfRectPrismFitting::captureThreadFunction()
{ 
  
  while (true)
  {

    // Lock before checking running flag
    boost::unique_lock<boost::mutex> capture_lock (capture_mutex);
    if(running)
    {
      pf->step(input, 0, false, -1);
      bestParticle=pf->getBest();
      
      // Check for shape slots
      if (num_slots<sig_cb_fitting_addapt> () > 0 )
        fitting_signal->operator() (getBestFit ());
    }
    capture_lock.unlock ();
  }
}


