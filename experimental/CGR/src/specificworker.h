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
#include <innermodel/innermodelviewer.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
        void LoadParameters();
	bool addPlane_notExisting(InnerModelViewer *innerViewer, const QString &item, const QString &base, const QVec &p, const QVec &n, const QString &texture, const QVec &size);
	void newFilteredPoints(const OrientedPoints &ops);
	void filterParticle();
        void drawParticles();
        bool removeNode(InnerModelViewer *innerViewer, const QString &item);
	void drawLines();
        bool updateInnerModel(InnerModel *innerModel);
        void updateLaser();
        void updateParticles();

public slots:
	void compute(); 	

private:
InnerModelViewer *innerModelViewer;
InnerModel *innerModel;
OsgView *osgView;	
VectorLocalization2D *localization;

string curMapName;
vector2f initialLoc;
vector2f curLoc;
float curAngle;
float initialAngle;
float locUncertainty, angleUncertainty;
VectorLocalization2D::MotionModelParams motionParams;
VectorLocalization2D::LidarParams lidarParams;
};

#endif

