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
#ifndef MSKSILHOUETTEI_H
#define MSKSILHOUETTEI_H

// QT includes
#include <QtCore/QObject>

// Ice includes
#include <Ice/Ice.h>
#include <MSKSilhouette.h>

#include <config.h>
#include "genericworker.h"

using namespace RoboCompMSKSilhouette;

class MSKSilhouetteI : public QObject , public virtual RoboCompMSKSilhouette::MSKSilhouette
{
Q_OBJECT
public:
	MSKSilhouetteI( GenericWorker *_worker, QObject *parent = 0 );
	~MSKSilhouetteI();
	void  getBodyList(BodyListData& body, const Ice::Current& = Ice::Current());
void  getRTMatrixList(const RoboCompMSKCommon::Id& identifier, JointList& jointl, const Ice::Current& = Ice::Current());
void  getBodyState(const RoboCompMSKCommon::Id& identifier, shapeType& shapet, const Ice::Current& = Ice::Current());
void  getNumBodies(NumBodiesData& nbodies, const Ice::Current& = Ice::Current());
void  getBody(const RoboCompMSKCommon::Id& identifier, TBody& body, const Ice::Current& = Ice::Current());
void  getJointPixelPosition(const JointData& data, RoboCompMSKCommon::Position2D& pxy, const Ice::Current& = Ice::Current());
void  pixelRGB2Dto3D(const RoboCompMSKCommon::Position2D& pxy, RoboCompMSKCommon::Position2D& traslation, const Ice::Current& = Ice::Current());
void  getRGB(RoboCompMSKCommon::TRGB& imgRGB, const Ice::Current& = Ice::Current());
void  getDepth(ImgDepthData& imgDepth, const Ice::Current& = Ice::Current());
void  getRGBD(RoboCompMSKCommon::TRGBD& imgRGBD, const Ice::Current& = Ice::Current());


	QMutex *mutex;
private:

	GenericWorker *worker;
public slots:


};

#endif