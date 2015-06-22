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
#include <map>
#include <vector>
#include <time.h>
#include <math.h>
#include <genericworker.h>
#include <omp.h>
#include "plane_filtering.h"

#define MAX_DEPTH 8000	
#define READ_WAIT_TIMEOUT 2000
#define PI 3.14159265359

struct TRadRotations{
  double rx;
  double ry;
  double rz;
};

//using namespace nite;
using namespace openni;
using namespace std;

struct DoubleBuffer
{
	//ATRIBUTOS DE LA struct
	QMutex bufferMutex;
	RoboCompRGBD::PointSeq bufferA;
	RoboCompRGBD::PointSeq *writer, *reader, *aux;
	int size;
	RoboCompRGBD::PointSeq bufferB;

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
		 	aux = writer;
			writer = reader;
			reader = aux;
		bufferMutex.unlock();
	}

	inline RoboCompRGBD::PointXYZ& operator[](int i){ return (*writer)[i]; };

	void copy(RoboCompRGBD::PointSeq &points)
	{
		points.resize(size);
		bufferMutex.lock();
			points = *reader;
		bufferMutex.unlock();
        }
};

class SpecificWorker : public GenericWorker
{        
Q_OBJECT
      ///ATRIBUTOS GENERALES
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
      RoboCompRGBD::DepthSeq* depthBuffer;
      RoboCompRGBD::ColorSeq* colorBuffer;
      imgType* colorImage;
      depthType* depthImage;

      ///MUTEX
      QMutex *usersMutex, *RGBMutex, *depthMutex, *pointsMutex;
      
      ///ATRIBUTOS PARA PINTAR       
      vector<short> normalDepth;
//      RCDraw *drawRGB;
      uint16_t *mColor;
      uint8_t *auxDepth;
      //IppiSize ippSizeImage;      
      QImage *qImgDepth;
      CoordinateConverter conversor;

      Registration registration;
      
      ///MÉTODOS PRIVADOS
      bool openDevice();
      bool openStream(SensorType sensorType, VideoStream *stream);
      void initializeStreams();
      void readFrame();
      void computeCoordinates();
      void readColor();
      void readDepth();
      
      void normalizeDepth();
      
      DoubleBuffer pointsBuff;
      RoboCompRGBD::DepthSeq * depthMapR, * depthMapW;
           
        
public:
        SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	
	///SERVANT: RGBD INTERFACE
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
	PlaneFilter *planeFilter;
	PointSeq points;
};

#endif

