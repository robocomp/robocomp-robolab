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
#include <QMat/QMatAll>

///if you use the last version of openni
// #include <openni/XnOpenNI.h>
// #include <openni/XnCodecIDs.h>
// #include <openni/XnCppWrapper.h>

///if you use the first version of openni
#include <ni/XnOpenNI.h>
#include <ni/XnCodecIDs.h>
#include <ni/XnCppWrapper.h>

/**
       \brief
       @author luiky
*/

#define CHECK(rc, what)											\
	if (rc != XN_STATUS_OK)											\
	{																\
		qDebug()<<what<<xnGetStatusString(nRetVal);					\
		\
	}	\
	else {															\
			qDebug()<<what<<xnGetStatusString(nRetVal);				\
	} 


#define SAMPLE_XML_PATH "Sample-User.xml"
#define POSE_TO_USE "Psi"

class SpecificWorker : public GenericWorker
{
Q_OBJECT
private:
	XnStatus nRetVal;
	
	xn::Context context;
	xn::DepthGenerator depthG;	
	xn::ImageGenerator imageG;
	xn::DepthMetaData depthMD;
// 	xn::ImageMetaData imageMD;	
	xn::UserGenerator userG;
	xn::Recorder recorder;
	xn::ScriptNode scriptNode;
	XnCallbackHandle hUserCallbacks, hCalibrationCallbacks, hPoseCallbacks;

	QMap<XnSkeletonJoint,QString> qmapNameJoints;	
	QMap<XnSkeletonJoint,XnSkeletonJointPosition> qmapXnJointPosition;
	QMap<XnSkeletonJoint,QVec> qmapQVecPosition;
	QMap<XnSkeletonJoint,RTMat> qmapRTMatJoints;
	RoboCompHumanTracker::RTMatrixList RTMatList;
	RoboCompHumanTracker::jointListType jointL;
	RoboCompHumanTracker::trackingState stateTrack;
	
	
	//Configuration methods for OpeNI
	void initializeNI();
	
	static void XN_CALLBACK_TYPE callBackNewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie);
	void newUser(xn::UserGenerator generator, XnUserID nId);
	
	static void XN_CALLBACK_TYPE callBackLostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie);
	void XN_CALLBACK_TYPE lostUser(xn::UserGenerator& generator, XnUserID nId);
	
	static void XN_CALLBACK_TYPE callBackPoseDetected(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie);
	void poseDetected(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId);
	
	static void XN_CALLBACK_TYPE callBackCalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie);
	void calibrationStart(xn::SkeletonCapability& capability, XnUserID nId);
	
	static void XN_CALLBACK_TYPE callBackCalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie);
	void calibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess);
	XnStatus openNiErrors(XnStatus nRet, QString what, xn::EnumerationErrors* errors );

	
	//utils
	void printFPS();
	
public:
	SpecificWorker(MapPrx& mprx, QObject *parent = 0);	
	~SpecificWorker();
	void setParams(RoboCompCommonBehavior::ParameterList params);
	

//Intefaces
private:
	RoboCompRGBD::DepthSeq depthMap;	
	RoboCompRGBD::ColorSeq colorMap;

public:
//----------------------
//RGDD Interface
//----------------------
	TRGBDParams getRGBDParams();
void setRegistration (const Registration& value);
Registration getRegistration ();
void getData(imgType& rgbMatrix, depthType& distanceMatrix, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState);
void getDepthInIR(depthType& distanceMatrix, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState);
void getImage(ColorSeq& color, DepthSeq& depth, PointSeq& points, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState);
void getDepth(DepthSeq& depth, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState);
void getRGB(ColorSeq& color, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState);
void getXYZ(PointSeq& points, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState);

//----------------------
//HumanTracker Interface
//----------------------
trackingState getState();
void  getRTMatrixList(RTMatrixList& RTMatList);
void  getJointsPosition(jointListType& jointList);
void  getData(RTMatrixList& RTMatList, jointListType& jointList, trackingState& state);


public slots:
 	void compute(); 	
};

#endif