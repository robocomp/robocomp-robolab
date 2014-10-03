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
#include "msksilhouetteI.h"

MSKSilhouetteI::MSKSilhouetteI(GenericWorker *_worker, QObject *parent) : QObject(parent)
{
	worker = _worker;
	mutex = worker->mutex;       // Shared worker mutex
	// Component initialization...
}


MSKSilhouetteI::~MSKSilhouetteI()
{
	// Free component resources here
}

// Component functions, implementation
void MSKSilhouetteI::getBodyList(BodyListData& body, const Ice::Current&){
	worker->getBodyList(body);
}

void MSKSilhouetteI::getRTMatrixList(const RoboCompMSKCommon::Id& identifier, JointList& jointl, const Ice::Current&){
	worker->getRTMatrixList(identifier,jointl);
}

void MSKSilhouetteI::getBodyState(const RoboCompMSKCommon::Id& identifier, shapeType& shapet, const Ice::Current&){
	worker->getBodyState(identifier,shapet);
}

void MSKSilhouetteI::getNumBodies(NumBodiesData& nbodies, const Ice::Current&){
	worker->getNumBodies(nbodies);
}

void MSKSilhouetteI::getBody(const RoboCompMSKCommon::Id& identifier, TBody& body, const Ice::Current&){
	worker->getBody(identifier,body);
}

void MSKSilhouetteI::getJointPixelPosition(const JointData& data, RoboCompMSKCommon::Position2D& pxy, const Ice::Current&){
	worker->getJointPixelPosition(data,pxy);
}

void MSKSilhouetteI::pixelRGB2Dto3D(const RoboCompMSKCommon::Position2D& pxy, RoboCompMSKCommon::Position2D& traslation, const Ice::Current&){
	worker->pixelRGB2Dto3D(pxy,traslation);
}

void MSKSilhouetteI::getRGB(RoboCompMSKCommon::TRGB& imgRGB, const Ice::Current&){
	worker->getRGB(imgRGB);
}

void MSKSilhouetteI::getDepth(ImgDepthData& imgDepth, const Ice::Current&){
	worker->getDepth(imgDepth);
}

void MSKSilhouetteI::getRGBD(RoboCompMSKCommon::TRGBD& imgRGBD, const Ice::Current&){
	worker->getRGBD(imgRGBD);
}


