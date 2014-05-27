#include "rectprismCloudParticle.h"


RectPrismCloudParticle::RectPrismCloudParticle(): r()
{
  this->weight=1;
//   varianceA=QVec::vec3(0, 0, 0);
//   varianceB=QVec::vec3(0, 0, 0);
//   varianceR=0;
  varianceC=QVec::vec3(10, 10, 10);
  varianceW=QVec::vec3(10, 10, 10);
  varianceR=QVec::vec3(0.1, 0.1, 0.1);
}

void RectPrismCloudParticle::estimateEigenAndCentroid(const RectPrismCloudPFInputData &data, Eigen::Vector3f &eig_values, Eigen::Matrix3f &eig_vectors, Eigen::Vector4f &centroid)
{
  static Eigen::Vector4f mean;
  static Eigen::Matrix3f cov;
  int np;
  
  centroid(0) = centroid(1) = centroid(2) = centroid(3) = 0.;
  np = 0;
  for (uint i=0; i<data.cloud_target->points.size(); i++)
  {
    if (isnan(data.cloud_target->points[i].z)) continue;
    centroid(0) += data.cloud_target->points[i].x;
    centroid(1) += data.cloud_target->points[i].y;
    centroid(2) += data.cloud_target->points[i].z;
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
  for (uint i=0; i<data.cloud_target->points.size(); i++)
  {
    tmp(0) = tmp(1) = tmp(2) = 0;
    if (isnan(data.cloud_target->points[i].z)) continue;
    tmp(0) = data.cloud_target->points[i].x - centroid(0);
    tmp(1) = data.cloud_target->points[i].y - centroid(1);
    tmp(2) = data.cloud_target->points[i].z - centroid(2);
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


void RectPrismCloudParticle::initializeFromEigenValues(const RectPrismCloudPFInputData &data){
  Eigen::Vector4f centroid;
  Eigen::Matrix3f covariance_matrix;
  
  pcl::compute3DCentroid (*data.cloud_target,centroid);
  computeCovarianceMatrix(*data.cloud_target,centroid,covariance_matrix);

 
  printf (" Size | %lu | \n", data.cloud_target->size());
  printf (" Centroid  | %f %f %f | \n", centroid (0), centroid (1), centroid (2));
  printf (" Covariance  | %f %f %f | \n", covariance_matrix (0,0), covariance_matrix (0,1), covariance_matrix (0,2));
  printf ("             | %f %f %f | \n", covariance_matrix (1,0), covariance_matrix (1,1), covariance_matrix (1,2));
  printf ("             | %f %f %f | \n", covariance_matrix (2,0), covariance_matrix (2,1), covariance_matrix (2,2));

  EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
  EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
//   pcl::eigen33(covariance_matrix, eigen_vectors, eigen_values);
  
  estimateEigenAndCentroid(data, eigen_values, eigen_vectors, centroid);

  printf (" EigenValues  | %f %f %f  | \n", eigen_values(0), eigen_values(1), eigen_values(2));
  printf (" EigenVectors | %f %f %f | \n", eigen_vectors (0,0), eigen_vectors (0,1), eigen_vectors (0,2));
  printf ("              | %f %f %f | \n", eigen_vectors (1,0), eigen_vectors (1,1), eigen_vectors (1,2));
  printf ("              | %f %f %f | \n", eigen_vectors (2,0), eigen_vectors (2,1), eigen_vectors (2,2));

  // Extract max values and indices.
  float max_value = NAN;
  int max_position = -1; 
  float second_max_value = NAN;
  int second_max_position = -1;
  for (int i=0; i<3; i++)
  {
    if (eigen_values(i) > max_value || isnan(max_value) )
    {
      second_max_value=max_value;
      second_max_position=max_position;
      max_value=eigen_values(i);
      max_position=i;
    }
    else if (eigen_values(i)  > second_max_value || isnan(second_max_value))
    {
      second_max_value=eigen_values(i);
      second_max_position=i;
    }
  }
  
  //use the biggest eigen value
  float max_eigenvalue=0;
  if(max_eigenvalue - eigen_values(0) < 0)
    max_eigenvalue=eigen_values(0);
  if(max_eigenvalue - eigen_values(1) < 0)
    max_eigenvalue=eigen_values(1);
  if(max_eigenvalue - eigen_values(2) < 0)
    max_eigenvalue=eigen_values(2);
  
  float max_distance=0;
  for (pcl::PointCloud<pcl::PointXYZRGB>::iterator it = data.cloud_target->points.begin (); it < data.cloud_target->points.end (); ++it)
  {
    float distance=fabs(RectPrism::distance_p2p(it->x, it->y, it->z, centroid(0), centroid(1), centroid(2)));
    if (max_distance<distance)
      max_distance=distance;
  }
  cout<<"RectPrismCloudParticle::initializeFromEigenValues::max_distance: "<<max_distance<<endl;

  float ratio=max_eigenvalue/max_distance;
  cout<<"RectPrismCloudParticle::initializeFromEigenValues::Ratio: "<<ratio<<endl;
  r.setCenter(QVec::vec3(centroid(0), centroid(1), centroid(2)));
  r.setWidth(QVec::vec3((eigen_values(0)/ratio)*2,(eigen_values(1)/ratio)*2,(eigen_values(2)/ratio))*2);
  float rx = atan2(eigen_vectors(2,1), eigen_vectors(2,2));
  float ry = atan2(-eigen_vectors(2,0),sqrt(pow(eigen_vectors(2,1),2)+pow(eigen_vectors(2,2),2)));
  float rz = atan2(eigen_vectors(1,0),eigen_vectors(0,0));
  r.setRotation(QVec::vec3(rx,ry,rz));
   
  
  // Extract endpoints
//   Vector max(eigen_vectors(max_position,0)*eigen_values(max_position),
//              eigen_vectors(max_position,1)*eigen_values(max_position),
//              eigen_vectors(max_position,2)*eigen_values(max_position));
//   Vector center(centroid(0), centroid(1), centroid(2));
// 
//   Vector a = center + max;
//   Vector b = center - max;
//   double r = eigen_values(second_max_position)/2;
  

//   printf("max:    (%f, %f, %f)\n", max[0], max[1], max[2]);
//   printf("center: (%f, %f, %f)\n", center[0], center[1], center[2]);
//   printf("a:      (%f, %f, %f)\n", a[0], a[1], a[2]);
//   printf("b:      (%f, %f, %f)\n", b[0], b[1], b[2]);
//   printf("r:      %f\n\n", r);
  // Set data
//   c.setValues(a, b, r);

}

void RectPrismCloudParticle::gypsyInitization(const RectPrismCloudPFInputData &data)
{
   Eigen::Vector4f centroid;
   pcl::compute3DCentroid (*data.cloud_target,centroid);
//    Vector center (centroid(0), centroid(1), centroid(2));
//    Vector a (centroid(0), centroid(1) + 50, centroid(2));
//    Vector b (centroid(0), centroid(1) - 50, centroid(2));
   
   r.setCenter(QVec::vec3(centroid(0), centroid(1), centroid(2)));
   r.setWidth(QVec::vec3(50,50,50));
   r.setRotation(QVec::vec3(0,0,0));

}

void RectPrismCloudParticle::initialize(const RectPrismCloudPFInputData &data, const int &control, const RCParticleFilter_Config *config)
{ 
   initializeFromEigenValues(data);
//  gypsyInitization(data);
}

void RectPrismCloudParticle::adapt(const int &controlBack, const int &controlNew, const bool noValidCandidates)
{
//   Vector a(c.getA().getX()+getRandom(varianceA(0)), c.getA().getY()+getRandom(varianceA(1)), c.getA().getZ()+getRandom(varianceA(2)));
//   Vector b(c.getB().getX()+getRandom(varianceB(0)), c.getB().getY()+getRandom(varianceB(1)), c.getB().getZ()+getRandom(varianceB(2)));
//   double r = fabs(c.getR()+getRandom(varianceR));
//   c.setValues(a,b,r);
  
  QVec currentCenter = r.getCenter();
  QVec currentWidth = r.getWidth();
  QVec currentRotation = r.getRotation();
  
  r.setCenter(QVec::vec3(currentCenter(0)+getRandom(varianceC(0)), currentCenter(1)+getRandom(varianceC(1)), currentCenter(2)+getRandom(varianceC(2))));
  r.setWidth(QVec::vec3(currentWidth(0)+getRandom(varianceW(0)), currentWidth(1)+getRandom(varianceW(1)), currentWidth(2)+getRandom(varianceW(2))));
  r.setRotation(QVec::vec3(currentRotation(0)+getRandom(varianceR(0)), currentRotation(1)+getRandom(varianceR(1)), currentRotation(2)+getRandom(varianceR(2))));
   
  float annealing = 1;
  varianceC = varianceC.operator*(annealing);
  varianceW = varianceW.operator*(annealing);
  varianceR = varianceR.operator*(annealing);
}

void RectPrismCloudParticle::computeWeight(const RectPrismCloudPFInputData &data)
{
//   printf("RectPrism: A(%f,%f,%f), B(%f,%f,%f), r=%f\n", c.getA().getX() , c.getA().getY() ,c.getA().getZ() , c.getB().getX() ,c.getB().getY() ,c.getB().getZ(), c.getR());
  this->weight=0.;
  //double mint, maxt;
  for( pcl::PointCloud<pcl::PointXYZRGB>::iterator it = data.cloud_target->begin(); it != data.cloud_target->end(); it++ )
  {
    if(!pcl_isnan(it->x) && !pcl_isnan(it->y) && !pcl_isnan(it->z))
    {
    
    QVec point = QVec::vec3(it->x, it->y, it->z);
//     double dist,t;
//     const float distA = sqrt(c.R(point));   
//     
//     QVec qP = QVec::vec3(it->x, it->y, it->z);
//     QVec qA = QVec::vec3(c.getA().getX(), c.getA().getY(), c.getA().getZ());
//     QVec qB = QVec::vec3(c.getB().getX(), c.getB().getY(), c.getB().getZ());
//     
//     if (distA<0.0001)
//     {
//       const double distB = c.getR() - ((qP-qA).crossProduct(qP-qB)).norm2() / (qB-qA).norm2();
// //       printf("metodo 2: %f\n", distB);
//       dist = distB;
//     }
//     else
//     {
// //       printf("metodo 1: %f\n", distA);
//       dist = distA;
//     }
    double dist = r.distance(point);
    
    this->weight += dist;
     //std::cout<<"X:"<<it->x<<" Y:"<<it->y<<" Z:"<<it->z<<" D:"<<dist<<std::endl;
    }
    //look for the upper and low points
//     t=-((qA-qP).dotProduct(qB-qA))/fabs(pow((qB-qA).norm2(),2));
//     if (t<mint || it==data.cloud_target->begin())
//     {
//       mint=t;
//     }
//     if (t>maxt || it==data.cloud_target->begin())
//     {
//       maxt=t;
//     }
  }
  this->weight /= data.cloud_target->points.size();
  //const float k = fabs(0.-mint)+fabs(1.-maxt);
  //const float topbottom_weight = 1./(c.getR()*k+1);
  const float distance_weight = 1./(this->weight+1.);
//   std::cout<<"k: "<<k<<std::endl;
//   std::cout<<"topbottom_weight: "<<topbottom_weight<<" distance_weight: "<<distance_weight<<std::endl;
  this->weight=distance_weight;//*topbottom_weight;
//   printf("WEIGHT: %f\n", this->weight);
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
