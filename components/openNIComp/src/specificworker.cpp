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

//----
//CALL BACK ZONE
//----

void SpecificWorker::newUser(xn::UserGenerator generator, XnUserID nId)
{
	
	userG.GetPoseDetectionCap().StartPoseDetection(POSE_TO_USE, nId);	
}

	
void XN_CALLBACK_TYPE SpecificWorker::callBackNewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	printf("New User %d\n", nId);
	((SpecificWorker *)pCookie)->newUser(generator, nId); 
}

void SpecificWorker::lostUser(xn::UserGenerator& generator, XnUserID nId)
{

}
// Callback: An existing user was lost
void XN_CALLBACK_TYPE SpecificWorker::callBackLostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	printf("Lost user %d\n", nId);
}


void SpecificWorker::poseDetected(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId)
{
	userG.GetPoseDetectionCap().StopPoseDetection(nId);
	userG.GetSkeletonCap().RequestCalibration(nId, TRUE);
}


void XN_CALLBACK_TYPE SpecificWorker::callBackPoseDetected(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie)
{
	printf("Pose %s detected for user %d\n", strPose, nId);
	((SpecificWorker *)pCookie)->poseDetected(capability,strPose, nId); 
	

}

void SpecificWorker::calibrationStart(xn::SkeletonCapability& capability, XnUserID nId)
{

}

void XN_CALLBACK_TYPE SpecificWorker::callBackCalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie)
{
	printf("Calibration started for user %d\n", nId);
}

void SpecificWorker::calibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess)
{
	if (bSuccess)
	{
		// Calibration succeeded
		printf("Calibration complete, start tracking user %d\n", nId);
		userG.GetSkeletonCap().StartTracking(nId);
	}
	else
	{
		// Calibration failed
		printf("Calibration failed for user %d\n", nId);
		userG.GetPoseDetectionCap().StartPoseDetection(POSE_TO_USE, nId);		
	}
}
void XN_CALLBACK_TYPE SpecificWorker::callBackCalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie)
{
	((SpecificWorker *)pCookie)->calibrationEnd(capability,nId,bSuccess);
}

//-------------
// Configuration
//----------
void SpecificWorker::initializeNI()
{
	nRetVal = XN_STATUS_OK;
	xn::EnumerationErrors errors;
	
	
	qDebug()<<"\nLoading OPENNI from XML: \n";
	//create node context	
	nRetVal =context.InitFromXmlFile(SAMPLE_XML_PATH, scriptNode,&errors);
	
	if (nRetVal != XN_STATUS_OK)
	{
		xnPrintError(nRetVal,"explicacion de OpenNI para este error: ");
		XnChar str [1024];
		errors.ToString(str,1024);
		qDebug()<<"info errors:"<<QString::fromStdString(str);
		qFatal("qFatal: aborted.");
	}
	nRetVal =context.FindExistingNode(XN_NODE_TYPE_DEPTH, depthG);
	CHECK(nRetVal, "Find DEPTH generator");
	
	nRetVal =context.FindExistingNode(XN_NODE_TYPE_IMAGE, imageG);
	CHECK(nRetVal, "Find IMAGE generator");
	
	nRetVal =context.FindExistingNode(XN_NODE_TYPE_USER, userG);
	CHECK(nRetVal, "Find USER generator");
		
	if (nRetVal==XN_STATUS_OK )
	{
		qDebug()<<"aa";
		if (!userG.IsCapabilitySupported(XN_CAPABILITY_SKELETON) || !userG.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
		{
			qFatal("User generator doesn't support either skeleton or pose detection.\n");		
		}
		userG.RegisterUserCallbacks(callBackNewUser, callBackLostUser, this, hUserCallbacks);
		userG.GetPoseDetectionCap().RegisterToPoseCallbacks(callBackPoseDetected, NULL, this, hPoseCallbacks);
		userG.GetSkeletonCap().RegisterCalibrationCallbacks(callBackCalibrationStart,callBackCalibrationEnd, this, hCalibrationCallbacks);	
		userG.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);
	}
	
	nRetVal =context.FindExistingNode(XN_NODE_TYPE_RECORDER, recorder);
	CHECK(nRetVal, "Find RECORDER generator");
	
	nRetVal = context.StartGeneratingAll();
	CHECK(nRetVal,"context.StartGeneratingAll()");	
	qDebug()<<"--------------------\n";
}


/**
* \brief Default constructor
*/

SpecificWorker::SpecificWorker(MapPrx& mprx, QObject *parent) : GenericWorker(mprx, parent)	
{
	initializeNI();
	colorMap.resize( XN_VGA_X_RES * XN_VGA_Y_RES );
	depthMap.resize( XN_VGA_X_RES * XN_VGA_Y_RES );
	this->setPeriod(33);
	
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}
void SpecificWorker::compute( )
{
	nRetVal = context.WaitAnyUpdateAll();			
	
	if (nRetVal != XN_STATUS_OK) {
		xnPrintError(nRetVal,"explicacion de OpenNI para este error: ");
	}
	
	const XnDepthPixel* depthTmp = depthG.GetDepthMap();
	mutex->lock();
	for( int y=0 ; y<XN_VGA_Y_RES ; y++ ) {
		for( int x=0 ; x<XN_VGA_X_RES ; x++ ) {
			const int offset = y*XN_VGA_X_RES + x;
			const float z = float(depthTmp[offset]) / 1000.0;
			if( z < 0.1 ) {
				depthMap[offset] = NAN;
			} else {
				depthMap[offset] = z;
			}
		}
	}

	memcpy(&colorMap[0],imageG.GetRGB24ImageMap(),XN_VGA_X_RES*XN_VGA_Y_RES*3);
	mutex->unlock();
	printFPS();
}

void SpecificWorker::printFPS()
{
	static int fps=0;
	static QTime ti(0,0,0);
	static QTime tt=QTime::currentTime();
	static QTime foo(0,0,0);
	if ((++fps % 50) == 0) {
		uint32_t e=ti.restart();
		if (e) {
			int v = 50000 / e;
			qDebug() << "Openni Fps: " << v << "Elapsed:" << foo.addMSecs( tt.elapsed()).toString("hh:mm:ss:zzz");
		}
	}
}
void SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
}

// ------------------------------------------------------------------------------------------------------------------------
// RGBD interface
// ------------------------------------------------------------------------------------------------------------------------
TRGBDParams SpecificWorker::getRGBDParams(){
	// TODO: implementar
	return TRGBDParams();
}
void SpecificWorker::setRegistration (const Registration& value){
	// TODO: implementar
	qDebug()<<"Warning: setRegistration (const Registration& value)";
	qDebug()<<"Not implemented yet.";
}
Registration SpecificWorker::getRegistration (){
	// TODO: implementar
	return RoboCompRGBD::DepthInColor;
}
void SpecificWorker::getData(imgType& rgbMatrix, depthType& distanceMatrix, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState){
	// TODO: implementar
	qDebug()<<"Warning: getData(imgType& rgbMatrix, depthType& distanceMatrix, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState)";
	qDebug()<<"Not implemented yet.";
}
void SpecificWorker::getDepthInIR(depthType& distanceMatrix, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState){
}
void SpecificWorker::getImage(ColorSeq& color, DepthSeq& depth, PointSeq& points, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState){
	// TODO: implementar
	qDebug()<<"Warning: getImage(ColorSeq& color, DepthSeq& depth, PointSeq& points, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState)";
	qDebug()<<"Not implemented yet.";
}
void SpecificWorker::getDepth(DepthSeq& depth, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState){
	mutex->lock();
	qDebug()<<"getDepth";
	depth=depthMap;
	mutex->unlock();
}
void SpecificWorker::getRGB(ColorSeq& color, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState){
	mutex->lock();
	color=colorMap;
	mutex->unlock();
}
void SpecificWorker::getXYZ(PointSeq& points, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState){
	// TODO: implementar
	qDebug()<<"Warning: getXYZ(PointSeq& points, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState)";
	qDebug()<<"Not implemented yet.";
	
}

// ------------------------------------------------------------------------------------------------------------------------
// HumanTracker interface
// ------------------------------------------------------------------------------------------------------------------------

trackingState SpecificWorker::getState(){
	return stateTrack;
}
void SpecificWorker::getRTMatrixList(RTMatrixList& RTMatList){
}
void SpecificWorker::getJointsPosition(jointListType& jointList){
}
void SpecificWorker::getData(RTMatrixList& RTMatList, jointListType& jointList, trackingState& state){
}
void hola() {
}
