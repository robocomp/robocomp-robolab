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

#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/transforms.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/openni_camera/openni_image.h>
#include <pcl/io/pcd_io.h>

#include "shapes/rectprism.h"
#include "fitting/pf_rect_prism_fitting.h"
#include "fitting/naive_rect_prism_fitting.h"
#include "visual/viewer.h"
#include <iostream>

/**
       \brief
       @author authorname
*/
typedef pcl::PointXYZRGBA PointT;
  
class SpecificWorker : public GenericWorker
{
	pcl::PointCloud<PointT>::Ptr cloud;
	pcl::PointCloud<PointT>::Ptr final_;

	bool first;
	bool new_cloud_available_flag;

	naiveRectangularPrismFitting* fitter;
	boost::shared_ptr<Viewer> vvv;


	RoboCompRGBD::ColorSeq rgbMatrix;
	RoboCompRGBD::depthType distanceMatrix;
	RoboCompRGBD::PointSeq points_kinect;
	RoboCompJointMotor::MotorStateMap h;
	RoboCompDifferentialRobot::TBaseState b;

Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();

	void translateClouds(pcl::PointCloud<PointT>::Ptr c_dest, const pcl::PointCloud<PointT>::ConstPtr &c_org );
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void fit_cb(const boost::shared_ptr<RectPrism>  &shape);

public slots:
	void compute(); 	
};
#endif
