#include "rect_prism_cloud_particle.h"


RectPrismCloudParticle::RectPrismCloudParticle(): r()
{
  this->weight=1;
//   varianceA=QVec::vec3(0, 0, 0);
//   varianceB=QVec::vec3(0, 0, 0);
//   varianceR=0;
//   varianceC=QVec::vec3(1000, 1000, 1000);
//   varianceW=QVec::vec3(1000, 1000, 1000);
//   varianceR=QVec::vec3(0.15, 0.15, 0.15);
  varianceC=QVec::vec3(50, 50, 50);
  varianceW=QVec::vec3(50, 50, 50);
  varianceR=QVec::vec3(0.5, 0.5, 0.5);
}

void RectPrismCloudParticle::estimateEigenAndCentroid(const RectPrismCloudPFInputData &data, Eigen::Vector3f &eig_values, Eigen::Matrix3f &eig_vectors, Eigen::Vector4f &centroid)
{
  static Eigen::Vector4f mean;
  static Eigen::Matrix3f cov;
  int np;
  
  centroid(0) = centroid(1) = centroid(2) = centroid(3) = 0.;
  np = 0;
  for (uint i=0; i<data.cloud_target.points.size(); i++)
  {
    if (isnan(data.cloud_target.points[i].z)) continue;
    centroid(0) += data.cloud_target.points[i].x;
    centroid(1) += data.cloud_target.points[i].y;
    centroid(2) += data.cloud_target.points[i].z;
    np += 1;
  }
  centroid(0) /= np;
  centroid(1) /= np;
  centroid(2) /= np;

  for (uint32_t ii=0 ; ii<3 ; ii++ )
    for (uint32_t jj=0 ; jj<3 ; jj++ )
      cov(ii,jj) = 0;

  Eigen::Vector4f tmp;
  tmp(3) = 0;
  for (uint i=0; i<data.cloud_target.points.size(); i++)
  {
    tmp(0) = tmp(1) = tmp(2) = 0;
    if (isnan(data.cloud_target.points[i].z)) continue;
    tmp(0) = data.cloud_target.points[i].x - centroid(0);
    tmp(1) = data.cloud_target.points[i].y - centroid(1);
    tmp(2) = data.cloud_target.points[i].z - centroid(2);
    for (uint32_t ii=0; ii<3; ii++)
      for (uint32_t jj=0; jj<3; jj++)
        cov(ii,jj) += tmp(ii)*tmp(jj);
  }
  cov /= np-1; 

  printf("x %f %f %f\n", centroid(0), centroid(1), centroid(2));
  printf("x %f %f %f\n", centroid(0), centroid(1), centroid(2));
  printf("x %f %f %f\n", centroid(0), centroid(1), centroid(2));
  printf("x %f %f %f\n", centroid(0), centroid(1), centroid(2));
  // Choose the column vector with smallest eigenvalue
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver( cov );
  eig_values = solver.eigenvalues();
  eig_vectors = solver.eigenvectors();
}


void RectPrismCloudParticle::initializeFromEigenValues(const RectPrismCloudPFInputData &data)
{
  
  Eigen::Vector4f centroid;
  Eigen::Matrix3f covariance_matrix;
  EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
  EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
  
  //calculate centroid, eigen values and eigen vectors
  pcl::computeMeanAndCovarianceMatrix(data.cloud_target, covariance_matrix, centroid);
  pcl::eigen33(covariance_matrix, eigen_vectors, eigen_values);
  
  //calculate ratio to resize the eigen_value 
  float max_distance=0;
  float max_eigenvalue=0;
  if(max_eigenvalue - eigen_values(0) < 0)
    max_eigenvalue=eigen_values(0);
  if(max_eigenvalue - eigen_values(1) < 0)
    max_eigenvalue=eigen_values(1);
  if(max_eigenvalue - eigen_values(2) < 0)
    max_eigenvalue=eigen_values(2);
  for (pcl::PointCloud<PointT>::const_iterator it = data.cloud_target.points.begin (); it < data.cloud_target.points.end (); ++it)
  {
    float distance=fabs(sqrt(pow((it->x)-centroid(0),2.0)+pow((it->y)-centroid(1),2.0)+pow((it->z)-centroid(2),2.0)));
    if (max_distance<distance)
      max_distance=distance;
  }
//   float ratio=max_eigenvalue/max_distance;
  
  //set initial values for the rectangular prism
  r.setCenter(QVec::vec3(centroid(0), centroid(1), centroid(2)));
  r.setWidth(QVec::vec3((eigen_values(0)/max_eigenvalue)*max_distance,(eigen_values(1)/max_eigenvalue)*max_distance,(eigen_values(2)/max_eigenvalue)*max_distance));

//   shape2Fit->setCenter(QVec::vec3(440,0,0));
//   shape2Fit->setWidth(QVec::vec3(100,100,400));
  
  float rx = atan2(eigen_vectors(2,1), eigen_vectors(2,2));
  float ry = atan2(-eigen_vectors(2,0),sqrt(pow(eigen_vectors(2,1),2)+pow(eigen_vectors(2,2),2)));
  float rz = atan2(eigen_vectors(1,0),eigen_vectors(0,0));
  r.setRotation(QVec::vec3(rx,ry,rz));

}

void RectPrismCloudParticle::initialize(const RectPrismCloudPFInputData &data, const int &control, const RCParticleFilter_Config *config)
{ 
   initializeFromEigenValues(data);
}

void RectPrismCloudParticle::adapt(const int &controlBack, const int &controlNew, const bool noValidCandidates)
{
  
  QVec currentCenter = r.getCenter();
  QVec currentWidth = r.getWidth();
  QVec currentRotation = r.getRotation();
  
  
  r.setCenter(QVec::vec3(currentCenter(0)+getRandom(varianceC(0)), currentCenter(1)+getRandom(varianceC(1)), currentCenter(2)+getRandom(varianceC(2))));
  r.setWidth(QVec::vec3(currentWidth(0)+getRandom(varianceW(0)), currentWidth(1)+getRandom(varianceW(1)), currentWidth(2)+getRandom(varianceW(2))));
  r.setRotation(QVec::vec3(currentRotation(0)+getRandom(varianceR(0)), currentRotation(1)+getRandom(varianceR(1)), currentRotation(2)+getRandom(varianceR(2))));
   
  float annealing = 0.96;
  varianceC = varianceC.operator*(annealing);
  varianceW = varianceW.operator*(annealing);
  varianceR = varianceR.operator*(annealing);
}

void RectPrismCloudParticle::computeWeight(const RectPrismCloudPFInputData &data)
{
  this->weight=0.;

//   int normalint =0;
  for( pcl::PointCloud<PointT>::const_iterator it = data.cloud_target.begin(); it != data.cloud_target.end(); it++ )
  {
    
    QVec point = QVec::vec3(it->x, it->y, it->z);

    double dist = r.distance(point);
    
    this->weight += dist;

  }
  this->weight /= data.cloud_target.points.size();
  const float distance_weight = 1./(this->weight+1.);

  this->weight=distance_weight;
}

RectPrismCloudParticle::~RectPrismCloudParticle()
{

}

QVec RectPrismCloudParticle::getTranslation()
{
  return r.getCenter();
}

QVec RectPrismCloudParticle::getRotation()
{
  return r.getRotation();
}

QVec RectPrismCloudParticle::getScale()
{
  return r.getWidth();
}

float RectPrismCloudParticle::getRandom(float var)
{
  double U = double(rand())/RAND_MAX;
  double V = double(rand())/RAND_MAX;
  return sqrt(-2*log(U))*cos(2.*M_PIl*V)*var;
}

void RectPrismCloudParticle::setRectPrism (RectPrism r )
{
  this->r.setCenter ( r.getCenter() );
  this->r.setRotation ( r.getRotation() );
  this->r.setWidth ( r.getWidth() );
  
}

void RectPrismCloudParticle::print(std::string v)
{
  printf("%s: \n", v.c_str());
  printf("RectPrism: Center (%f,%f,%f), Rotation (%f,%f,%f), Width (%f,%f,%f), Weight: %f\n", 
   r.getCenter()(0),r.getCenter()(1),r.getCenter()(2),
   r.getRotation()(0),r.getRotation()(1),r.getRotation()(2),
   r.getWidth()(0),r.getWidth()(1),r.getWidth()(2), weight);
}   
