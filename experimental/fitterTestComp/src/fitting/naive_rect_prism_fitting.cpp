#include "naive_rect_prism_fitting.h"

naiveRectangularPrismFitting::naiveRectangularPrismFitting( pcl::PointCloud<PointT>::Ptr cloud )
: shape2Fit(new RectPrism())
, bestFit(new RectPrism())
{  
  bestweight=999999;
  
  pointCloud2Fit=cloud; 
  
  captured_thread = boost::thread (&naiveRectangularPrismFitting::captureThreadFunction, this);
  fitting_signal = createSignal<sig_cb_fitting_addapt> ();
  
  //initialize Rectangular prism to cloud
  initRectangularPrism();
 
  //initialize DimensionChanged
  initDimensionChanged();
}

void naiveRectangularPrismFitting::initDimensionChanged()
{
  for(int i=0; i<9; i++)
  {
    dimensionChanged[i]=true;
  }
}

bool naiveRectangularPrismFitting::notAnyDimensionChanged()
{
  for(int i=0; i<9; i++)
  {
    if(dimensionChanged[i])
      return false;
  }
  return true;
}

void naiveRectangularPrismFitting::initRectangularPrism ()
{
  
  Eigen::Vector4f centroid;
  Eigen::Matrix3f covariance_matrix;
  EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
  EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
  
  //calculate centroid, eigen values and eigen vectors
  pcl::computeMeanAndCovarianceMatrix(*pointCloud2Fit, covariance_matrix, centroid);
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
  for (pcl::PointCloud<pcl::PointXYZRGBA>::iterator it = pointCloud2Fit->points.begin (); it < pointCloud2Fit->points.end (); ++it)
  {
    float distance=fabs(sqrt(pow((it->x)-centroid(0),2.0)+pow((it->y)-centroid(1),2.0)+pow((it->z)-centroid(2),2.0)));
    if (max_distance<distance)
      max_distance=distance;
  }
  float ratio=max_eigenvalue/max_distance;
  MAX_WIDTH=max_distance;
  
  //set initial values for the rectangular prism
  shape2Fit->setCenter(QVec::vec3(centroid(0), centroid(1), centroid(2)));
  shape2Fit->setWidth(QVec::vec3((eigen_values(0)/max_eigenvalue)*max_distance,(eigen_values(1)/max_eigenvalue)*max_distance,(eigen_values(2)/max_eigenvalue)*max_distance));

//   shape2Fit->setCenter(QVec::vec3(440,0,0));
//   shape2Fit->setWidth(QVec::vec3(100,100,400));
  
  float rx = atan2(eigen_vectors(2,1), eigen_vectors(2,2));
  float ry = atan2(-eigen_vectors(2,0),sqrt(pow(eigen_vectors(2,1),2)+pow(eigen_vectors(2,2),2)));
  float rz = atan2(eigen_vectors(1,0),eigen_vectors(0,0));
  shape2Fit->setRotation(QVec::vec3(rx,ry,rz));
}

void naiveRectangularPrismFitting::captureThreadFunction ()
{
  while (true)
  {

    // Lock before checking running flag
    boost::unique_lock<boost::mutex> capture_lock (capture_mutex);
    if(running)
    {
      adapt ();
      // Check for shape slots
      if (num_slots<sig_cb_fitting_addapt> () > 0 )
        fitting_signal->operator() (getRectangularPrism ());
      
      if(notAnyDimensionChanged() && weight > 0.1)
      {
        cout<<"RESET"<<endl;
        initRectangularPrism();
        initDimensionChanged();
      }
      
    }
    capture_lock.unlock ();
  }
}
  
float naiveRectangularPrismFitting::computeWeight()
{
  weight=0.;
  //estimate normals
//   pcl::NormalEstimation<PointT, pcl::Normal> ne;
//   ne.setInputCloud (pointCloud2Fit);
//   pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
//   ne.setSearchMethod (tree);
//   // Output datasets
//   pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
//   // Use all neighbors in a sphere of radius 3cm
//   ne.setRadiusSearch (5);
//   // Compute the features
//   ne.compute (*cloud_normals);

  //int normalint =0;
  for( pcl::PointCloud<PointT>::iterator it = pointCloud2Fit->begin(); it != pointCloud2Fit->end(); it++ )
  {
    
    QVec point = QVec::vec3(it->x, it->y, it->z);
    //QVec normal = QVec::vec3( cloud_normals->points[normalint].normal_x,cloud_normals->points[normalint].normal_y, cloud_normals->points[normalint].normal_z);


    //double dist = shape2Fit->distance(point,normal);
    double dist = shape2Fit->distance(point);
    weight += dist*dist;
    //normalint++;
  }
  
  weight /= pointCloud2Fit->points.size();

  return weight;
}

void naiveRectangularPrismFitting::adapt()
{
  
  switch(rand()%9)
  {
    //x
    case 0:
      incTranslation(0);
      break;
    //y
    case 1:
      incTranslation(1);
      break;
    //z
    case 2:
      incTranslation(2);
      break;
    //Wx
    case 3:
      incWidth(0);
      break;
    //Wy
    case 4:
      incWidth(1);
      break;  
    //Wz
    case 5:
      incWidth(2);
      break;
    //Rx
    case 6:
      incRotation(0);
      break;
    //Ry
    case 7:
      incRotation(1);
      break;
    //Rz
    case 8:
      incRotation(2);
      break;       
  }
}

void naiveRectangularPrismFitting::incTranslation(int index)
{
  QVec auxvec;
  float positiveWeight, negativeWeight, transformedWeight;
  QVec width = shape2Fit->getWidth();
  float inc = width(index)/10;
  auxvec = shape2Fit->getCenter();
  
  //decide direction
  auxvec(index)=auxvec(index)+inc;
  shape2Fit->setCenter(auxvec);
  computeWeight();
  positiveWeight=weight;
  
  //undo and sobstract a quarter = 2 quarters
  auxvec(index)=auxvec(index)-(inc*2);
  shape2Fit->setCenter(auxvec);
  computeWeight();
  negativeWeight=weight;
  
  //undo changes
  auxvec(index)=auxvec(index)+inc;
  shape2Fit->setCenter(auxvec);
  
  computeWeight();
  
    //cout<<"positive: "<<positiveWeight<<" negative: "<<negativeWeight<<endl;
  //if negative is good go with it
  if(negativeWeight<positiveWeight)
  {
    transformedWeight=negativeWeight;
    inc*=-1;
  }
  else
    transformedWeight=positiveWeight;
    
  if(transformedWeight<weight)
    dimensionChanged[index]=true;
  else
    dimensionChanged[index]=false;
  
  //cout<<"Transformed: "<<transformedWeight<<" weight: "<<weight<<endl;
  while(transformedWeight<weight)
  {
//     cout<<"INCT: "<<inc<<endl;
    auxvec(index)=auxvec(index)+inc;
    shape2Fit->setCenter(auxvec);
    weight=transformedWeight;
    
    //calculate futureWeight
    auxvec(index)=auxvec(index)+inc;
    shape2Fit->setCenter(auxvec);
    computeWeight();
    transformedWeight=weight;
    
    //undo
    auxvec(index)=auxvec(index)-inc;
    shape2Fit->setCenter(auxvec);
    computeWeight();
  }
  
  if(weight<bestweight)
  {
    bestFit->setCenter(shape2Fit->getCenter());
    bestFit->setRotation(shape2Fit->getRotation());
    bestFit->setWidth(shape2Fit->getWidth());
    bestweight=weight;
  }

}

void naiveRectangularPrismFitting::incRotation(int index)
{
  QVec auxvec;
  float positiveWeight, negativeWeight, transformedWeight;
  QVec width = shape2Fit->getWidth();
  float inc = 0.02;
  auxvec = shape2Fit->getRotation();
  
  //decide direction
  auxvec(index)=auxvec(index)+inc;
  shape2Fit->setRotation(auxvec);
  computeWeight();
  positiveWeight=weight;
  
  //undo and sobstract a quarter = 2 quarters
  auxvec(index)=auxvec(index)-(inc*2);
  shape2Fit->setRotation(auxvec);
  computeWeight();
  negativeWeight=weight;
  
  //undo changes
  auxvec(index)=auxvec(index)+inc;
  shape2Fit->setRotation(auxvec);
  computeWeight();
  
  //if negative is good go with it
  if(negativeWeight<positiveWeight)
  {
    transformedWeight=negativeWeight;
    inc*=-1;
  }
  else
    transformedWeight=positiveWeight;
    
  if(transformedWeight<weight && auxvec(index)+inc <= MAX_WIDTH)
    dimensionChanged[index+3]=true;
  else
    dimensionChanged[index+3]=false;
  
  while(transformedWeight<weight && auxvec(index)+inc <= MAX_WIDTH)
  {
//     cout<<"INCR: "<<inc<<endl;
    auxvec(index)=auxvec(index)+inc;
    shape2Fit->setRotation(auxvec);
    weight=transformedWeight;
    
    //calculate futureWeight
    auxvec(index)=auxvec(index)+inc;
    shape2Fit->setRotation(auxvec);
    computeWeight();
    transformedWeight=weight;
    
    //undo
    auxvec(index)=auxvec(index)-inc;
    shape2Fit->setRotation(auxvec);
    computeWeight();
  }
  
  if(weight<bestweight)
  {
    bestFit->setCenter(shape2Fit->getCenter());
    bestFit->setRotation(shape2Fit->getRotation());
    bestFit->setWidth(shape2Fit->getWidth());
    bestweight=weight;
  }
}

void naiveRectangularPrismFitting::incWidth(int index)
{
  float positiveWeight, negativeWeight, transformedWeight;
  QVec auxvec = shape2Fit->getWidth();
  float inc = auxvec(index)/10;
  
  //decide direction
  auxvec(index)=auxvec(index)+inc;
  shape2Fit->setWidth(auxvec);
  computeWeight();
  positiveWeight=weight;
  
  //undo and sobstract a quarter = 2 quarters
  auxvec(index)=auxvec(index)-(inc*2);
  shape2Fit->setWidth(auxvec);
  computeWeight();
  negativeWeight=weight;
  
  //undo changes
  auxvec(index)=auxvec(index)+inc;
  shape2Fit->setWidth(auxvec);
  computeWeight();
  
  //if negative is good go with it
  if(negativeWeight<positiveWeight)
  {
    transformedWeight=negativeWeight;
    inc*=-1;
  }
  else
    transformedWeight=positiveWeight;
    
  if(transformedWeight<weight)
    dimensionChanged[index+6]=true;
  else
    dimensionChanged[index+6]=false;
  
  while(transformedWeight<weight)
  {
//     cout<<"INCW: "<<inc<<endl;
    auxvec(index)=auxvec(index)+inc;
    shape2Fit->setWidth(auxvec);
    weight=transformedWeight;
    
    //calculate futureWeight
    auxvec(index)=auxvec(index)+inc;
    shape2Fit->setWidth(auxvec);
    computeWeight();
    transformedWeight=weight;
    
    //undo
    auxvec(index)=auxvec(index)-inc;
    shape2Fit->setWidth(auxvec);
    computeWeight();
  }
  
  if(weight<bestweight)
  {
    bestFit->setCenter(shape2Fit->getCenter());
    bestFit->setRotation(shape2Fit->getRotation());
    bestFit->setWidth(shape2Fit->getWidth());
    bestweight=weight;
  }
}