 /*
  *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
  *
  *    This file is part of RoboComp
  *
  *    RoboComp is free software: you can redistribute it and/or modify
  *    it under the terms of the GNU General Public License as published by
  *    the Free Software Foundation, either version 3 of the License, or
  *    (at your option) any later version.
  *
  *    RoboComp is distributed in the hope that it will be useful,
  *    but WITHOUT ANY WARRANTY; without even the implied warranty of
  *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  *    GNU General Public License for more details.
  *
  *    You should have received a copy of the GNU General Public License
  *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
  */
  
#include "specificworker.h"

/**
  * \brief Default constructor
  */
SpecificWorker::SpecificWorker(MapPrx& mprx, QObject *parent) : GenericWorker(mprx, parent)
, cloud(new pcl::PointCloud<PointT>)
, final_(new pcl::PointCloud<PointT>) 

, cloud_cup(new pcl::PointCloud<PointT>)
{
       //Sintetic cube
  int Wx = 100;
  int Wy = 100;
  int Wz = 500;
  int res = 3;
  //Rot3D r(0.5, 0.2, 0.2);
  //Faces front and back
  for(float x=0; x<=Wx; x=x+res)
  {
    for(float y=0; y<=Wx; y=y+res)
    {
      //face front (x=0)
      pcl::PointXYZRGB p;
      p.x = x;
      p.y = y;
      p.z = 0;
      cloud_cup->push_back(p);
      p.x = x;
      p.y = y;
      p.z = Wz;      
      cloud_cup->push_back(p);
    }
  }
  //Faces up and down
  for(float x=0; x<=Wx; x=x+res)
  {
    for(float z=0; z<=Wz; z=z+res)
    {
      //face front (x=0)
      pcl::PointXYZRGB p;
      p.x = x;
      p.y = 0;
      p.z = z;
      cloud_cup->push_back(p);
      p.x = x;
      p.y = Wy;
      p.z = z;      
      cloud_cup->push_back(p);
    }
  }
  //Faces right and left
  for(float y=0; y<=Wy; y=y+res)
  {
    for(float z=0; z<=Wz; z=z+res)
    {
      //face front (x=0)
      pcl::PointXYZRGB p;
      p.x = 0;
      p.y = y;
      p.z = z;
      cloud_cup->push_back(p);
      p.x = Wx;
      p.y = y;
      p.z = z;      
      cloud_cup->push_back(p);
    }
  }
  
 
  boost::shared_ptr<RectPrismFitting> rectprismf(new RectPrismFitting(cloud_cup));
  rectprismfitting=rectprismf;
}

/**
  * \brief Default destructor
  */
SpecificWorker::~SpecificWorker()
{
  
}

void SpecificWorker::compute()
{ 
  //ICE
 RoboCompInnerModelManager::PointCloudVector pointcloud;
  //Ransac


  std::vector<int> inliers;
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
  pcl::ExtractIndices<PointT> extract;

//   try{
//   rgbd_proxy->getImage(rgbMatrix, distanceMatrix, points_kinect,  h, b);
//   }catch(Ice::Exception e)
//   {
//     qDebug()<<"Error talking to rgbd_proxy: "<<e.what();
//   }
//   
//   for(int i=0; i<points_kinect.size(); i++)
//   {
//     PointT point;
//     point.x=points_kinect[i].x*1000; 
//     point.y=points_kinect[i].y*1000; 
//     point.z=points_kinect[i].z*1000; 
//     point.r=rgbMatrix[i].red;
//     point.g=rgbMatrix[i].green;
//     point.b=rgbMatrix[i].blue;
//     
//     cloud->push_back(point);
//   }
  
//   pcl::SampleConsensusModelPlane<PointT>::Ptr
//     model_s(new pcl::SampleConsensusModelPlane<PointT> (cloud));
//     
//   pcl::RandomSampleConsensus<PointT> ransac (model_s);
//   
//   ransac.setDistanceThreshold (0.1);
//   ransac.computeModel();
//   ransac.getInliers(inliers);
//   inliers_plane->indices=inliers;
//   
//   //Extract the rest
//   extract.setInputCloud (cloud);
//   extract.setIndices (inliers_plane);
//   extract.setNegative (true);
//   extract.filter(*final_);
//   
  pointcloud.resize(cloud_cup->size());
  int j=0;
  for (pcl::PointCloud<PointT>::iterator it = cloud_cup->points.begin (); it < cloud_cup->points.end (); ++it)
  {
//     pointcloud[j].r=it->r;
//     pointcloud[j].g=it->g;
//     pointcloud[j].b=it->b;
    pointcloud[j].r=0;
    pointcloud[j].g=0;
    pointcloud[j].b=255;
    pointcloud[j].x=(it->x);
    pointcloud[j].y=(it->y);
    pointcloud[j].z=(it->z);
    
    j++;
  }
  
  RoboCompInnerModelManager::Pose3D pose;
  QVec transform, rotation, width;
  
  try
  {
    innermodelmanager_proxy->setPointCloudData(string("cloud"), pointcloud);
  }catch(Ice::Exception e)
  {
    qDebug()<<"Error talking to inermodelmanager_proxy: "<<e.what();
  }

  rectprismfitting->run(cloud_cup, transform, rotation, width);
  pose.x=transform(0);
  pose.y=transform(1);
  pose.z=transform(2);
  pose.rx=rotation(0);
  pose.ry=rotation(1);
  pose.rz=rotation(2);
  
//  cout<<"POSE: "<<pose.x<<" "<<pose.y<<" "<<pose.z<<endl;
  
  try
  {
    innermodelmanager_proxy->setPoseFromParent("cube_0", pose);
    innermodelmanager_proxy->setScale("cube", width(0)/2.f, width(1)/2.f, width(2)/2.f);
  }catch(Ice::Exception e)
  {
    qDebug()<<"Error talking to inermodelmanager_proxy: "<<e.what();
  }
  

  
  cloud->clear();
  final_->clear();
}
void SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
  timer.setSingleShot(true);
  timer.start(Period);
};

pcl::PointCloud<PointT>::Ptr SpecificWorker::readPCLCloud(QString name)
{
  pcl::PCDReader reader;
  pcl::PointCloud<PointT>::Ptr cloud;
  if (reader.read(name.toStdString(), *cloud) != -1)
    qFatal("No file %s", name.toStdString().c_str());
  return cloud;
};


