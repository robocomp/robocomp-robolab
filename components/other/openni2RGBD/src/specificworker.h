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
#include <OpenNI.h>
#include <PS1080.h>
#include <map>
#include <vector>
#include <time.h>
#include <math.h>

/**
       \brief
       @author authorname
*/

#define MAX_DEPTH 8000	
#define READ_WAIT_TIMEOUT 2000
#define PI 3.14159265359

///DEFINICIONES TEMPORALES, A ESPERA DE INTERFAZ DE TRACKING
///TEMPORAL
struct TRadRotations{
  double rx;
  double ry;
  double rz;
};

//using namespace nite;
using namespace openni;
using namespace std;

template <class T> class DoubleBuffer
{
	QMutex bufferMutex;
	T bufferA, *writer, *reader, bufferB;
	public:
		int size;
		DoubleBuffer(){};
		void resize(int size_)
		{
			bufferA.resize(size_);
			writer = &bufferA;
			bufferB.resize(size_);
			reader = &bufferB;
			size = size_;
		}
		void swap()
		{
			bufferMutex.lock();
				writer->swap(*reader);
			bufferMutex.unlock();
		}

		inline typename T::value_type& operator[](int i){ return (*writer)[i]; };

		void copy(T &points)
		{
			points.resize(size);
			bufferMutex.lock();
			points = *reader;
			bufferMutex.unlock();
		}
		T* getWriter()
		{
		  return writer;
		}
};


class SpecificWorker : public GenericWorker
{
Q_OBJECT
	int IMAGE_WIDTH, IMAGE_HEIGHT;
	int fps; 

	openni::Status openniRc;
	Device device;
	VideoStream depth;
	VideoStream color;
	VideoFrameRef depthFrame;
	VideoFrameRef colorFrame;

	VideoStream* pStream;
	int changedStreamDummy;
	DepthPixel* pixDepth;
	RoboCompRGBD::DepthSeq* depthBuffer;
	RoboCompRGBD::ColorSeq* colorBuffer;
	imgType* colorImage;
	depthType* depthImage;
	
	///MUTEX
	QMutex *usersMutex, *RGBMutex, *depthMutex, *pointsMutex, *bStateMutex, *mStateMutex;
	
	vector<short> normalDepth;
	uint16_t *mColor;
	uint8_t *auxDepth;
	CoordinateConverter conversor;

	Registration registration;
	
	DoubleBuffer<RoboCompRGBD::PointSeq> pointsBuff;
	DoubleBuffer<RoboCompRGBD::DepthSeq> depthBuff;
	
	RoboCompGenericBase::TBaseState bState;
	RoboCompJointMotor::MotorStateMap mState;

	RoboCompRGBD::DepthSeq * depthMapR, * depthMapW;
	QMutex *worker_params_mutex;
	RoboCompCommonBehavior::ParameterList worker_params;
	bool talkToJoint,talkToBase,depthB,colorB;
	
	//------------method---------
	void openDevice();
	bool openStream(SensorType sensorType, VideoStream *stream);
	void initializeStreams();
	void checkInitialization();
	bool readFrame();
	void readDepth();
	void readColor();
	void computeCoordinates();
	void normalizeDepth();
	void closeStreams();

	
           
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	RoboCompCommonBehavior::ParameterList getWorkerParams();
	///RGBD INTERFACE
	TRGBDParams getRGBDParams( );
	void setRegistration (const RoboCompRGBD::Registration &value);
	Registration getRegistration ( );
	void getData(imgType& rgbMatrix, depthType& distanceMatrix, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState& bState);
	void getDepthInIR(depthType& distanceMatrix, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState& bState);
	void getImage(ColorSeq& color, DepthSeq& depth, PointSeq& points, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState& bState);
	void getDepth(DepthSeq& depth, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState& bState );
	void getRGB(ColorSeq& color, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState& bState);
	void getXYZ(PointSeq& points, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState& bState);

public slots:
 	void compute(); 	
};

#endif

