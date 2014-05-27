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
#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>

#ifndef Q_MOC_RUN
#include <pcl/point_cloud.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
#endif

#include <boost/thread/thread.hpp>
#include "rectprismFitting.h"



/**
       \brief
       @author authorname
*/

typedef pcl::PointXYZRGB PointT;

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx, QObject *parent = 0);	
	~SpecificWorker();
	void setParams(RoboCompCommonBehavior::ParameterList params);
  Vector V(const double& r);
  inline double getRandom() { return (rand()%32000)/32000.0; }
	
public slots:
	void compute(); 	
	
private:
  
  Vector p[10000];
  Vector d[10000];
  double t[10000];
  
  
  RoboCompRGBD::ColorSeq rgbMatrix;
  RoboCompRGBD::depthType distanceMatrix;
  RoboCompRGBD::PointSeq points_kinect;
  RoboCompJointMotor::MotorStateMap h;
  RoboCompDifferentialRobot::TBaseState b;

  
  pcl::PointCloud<PointT>::Ptr cloud;
  pcl::PointCloud<PointT>::Ptr final_;
  pcl::PointCloud<PointT>::Ptr cloud_cup;
  pcl::PointCloud<PointT>::Ptr readPCLCloud(QString name);
  
  boost::shared_ptr<RectPrismFitting> rectprismfitting;
};

#endif