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
#include <OpenNI.h>
#include <PS1080.h>

#define MAX_DEPTH 8000	
#define READ_WAIT_TIMEOUT 2000

using namespace openni;
using namespace std;

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	///RGBDBus INTERFACE
	virtual CameraParamsMap getAllCameraParams();
	virtual void getPointClouds(const CameraList &cameras, PointCloudMap &clouds);
	virtual void getImages(const CameraList &cameras, ImageMap &images);
	virtual void getProtoClouds(const CameraList &cameras, PointCloudMap &protoClouds);
	virtual void getDecimatedImages(const CameraList &cameras, const int decimation, ImageMap &images);
	
	///RGBD INTERFACE
	TRGBDParams getRGBDParams( );
	void setRegistration (RoboCompRGBD::Registration value);
	Registration getRegistration ( );
	void getData(imgType& rgbMatrix, depthType& distanceMatrix, RoboCompJointMotor::MotorStateMap &hState, RoboCompDifferentialRobot::TBaseState& bState);
	void getDepthInIR(depthType& distanceMatrix, RoboCompJointMotor::MotorStateMap &hState, RoboCompDifferentialRobot::TBaseState& bState);
	void getImage(ColorSeq& color, DepthSeq& depth, PointSeq& points, RoboCompJointMotor::MotorStateMap &hState, RoboCompDifferentialRobot::TBaseState& bState);
	void getDepth(DepthSeq& depth, RoboCompJointMotor::MotorStateMap &hState, RoboCompDifferentialRobot::TBaseState& bState );
	void getRGB(ColorSeq& color, RoboCompJointMotor::MotorStateMap &hState, RoboCompDifferentialRobot::TBaseState& bState);
	void getXYZ(PointSeq& points, RoboCompJointMotor::MotorStateMap &hState, RoboCompDifferentialRobot::TBaseState& bState);
	

public slots:
	void compute(); 	

private:
	int IMAGE_WIDTH, IMAGE_HEIGHT;
	
	///ATRIBUTOS NECESARIOS PARA LA LECTURA DE FLUJOS
	openni::Status openniRc;
	Device device;
	VideoStream depth;
	VideoStream color;
	VideoFrameRef depthFrame;
	VideoFrameRef colorFrame;
	
	///ATRIBUTOS PARA LA OBTENCIÓN DE COLOR Y PROFUNDIDAD CON OPENNI
	VideoStream* pStream;
	int changedStreamDummy;
	DepthPixel* pixDepth;
	
 	RoboCompRGBDBus::DepthBuffer depthBufferB, depthImageB;
 	RoboCompRGBDBus::ColorBuffer colorImageB;
	
	RoboCompRGBD::DepthSeq* depthBuffer;
	RoboCompRGBD::ColorSeq* colorBuffer;
	imgType* colorImage;
	depthType* depthImage;

	///MUTEX
	QMutex *usersMutex, *RGBMutex, *depthMutex;
	
	RoboCompRGBD::Registration registration;
	
	///MÉTODOS PRIVADOS
	void openDevice();
	bool openStream(SensorType sensorType, VideoStream *stream);
	void initializeStreams();
	void initializeTracking();
	void readFrame();
	void readColor();
	void readDepth();
	

};

#endif

