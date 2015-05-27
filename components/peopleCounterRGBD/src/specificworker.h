/*
 *    Copyright (C) 2015 by YOUR NAME HERE
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

/**
       \brief
       @author authorname
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <innermodel/innermodelviewer.h>
#include <osgviewer/osgview.h>
#include <nabo/nabo.h>

using namespace Nabo;
using namespace Eigen;


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);


public slots:
	void compute(); 	

private:
	InnerModel *innerModel;
	InnerModelViewer *innerModelViewer;
	OsgView 			*osgView;			
	IMVPointCloud *imvPointCloud;
	
  //libnabo	
	NNSearchF* nns;
	MatrixXf data;
	
	void updatePointCloud(const PointSeq &points);
	void storeBackground();
	void computeBackground(PointSeq& points);
	bool filterP( const osg::Vec3f& p, const PointXYZ& point);
	
};

#endif

