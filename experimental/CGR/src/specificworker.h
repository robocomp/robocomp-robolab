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

#include "vectorparticlefilter.h"
#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <osgviewer/osgview.h>
#include "innermodeldraw.h"
#include <innermodel/innermodelviewer.h>
#include <math.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
        void LoadParameters();
	void newFilteredPoints(const OrientedPoints &ops);
	void filterParticle();
        void drawParticles();
	void drawLines();
        void updateLaser();
        void updateParticles();
	void resetPose(const float x, const float z, const float alpha);
	float cgrCertainty();
public slots:
	void compute();
private:
	InnerModelViewer *innerModelViewer;
	InnerModel *innerModel;
	OsgView *osgView;	
	VectorLocalization2D *localization;
	QTime t;
	string curMapName;
	vector2f initialLoc;
	vector2f curLoc, curLocOld;
	float curAngle;
	float initialAngle;
	float locUncertainty, angleUncertainty;
	VectorLocalization2D::MotionModelParams motionParams;
	VectorLocalization2D::LidarParams lidarParams;
};

#endif

