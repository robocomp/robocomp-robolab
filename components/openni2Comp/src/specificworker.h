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
#include <NiTE.h>
#include <map>
#include <vector>
#include <ipp.h>
#include <rcdraw/rcdraw.h>
#include <time.h>
#include <math.h>

/**
       \brief
       @author authorname
*/

#define MAX_DEPTH 8000	
#define READ_WAIT_TIMEOUT 2000
#define MAX_USERS 16
#define NUM_JOINT 16 ///LOS DE NITE+CINTURA
#define NUM_ROTATIONS 11	///NI CABEZA, NI MANOS NI PIES, SI CINTURA
#define TAM_JOINT 4
#define TAM_ROTATION 6
#define PI 3.14159265359
#define USER_MESSAGE(msg) \
	{cout << "[" << ts << "] User #" << user.getId() << ": "<< msg << endl;}

///DEFINICIONES TEMPORALES, A ESPERA DE INTERFAZ DE TRACKING
/*typedef nite::SkeletonState trackingState;

struct TPerson{			//Tipo Persona
  trackingState state; 		//Estado de un usuario
  jointListType joints;		//Lista posiciones del cuerpo
  RTMatrixList rotations;    	//Lista Rotaciones
};
//Lista de Usuarios
typedef map <short int,TPerson> PersonList; //NO VECTOR PARA NO DEVOLVER TODOS LOS USUARIOS, Y CON ID ALTERNADOS
*/
///TEMPORAL
struct TRadRotations{
  double rx;
  double ry;
  double rz;
};

using namespace nite;
using namespace openni;
using namespace std;

typedef enum{
	NOT_CAPTURING,
	SHOULD_CAPTURE,
	CAPTURING,
} CapturingState;
///http://www.openni.org/openni-programmers-guide/


class SpecificWorker : public GenericWorker
{
Q_OBJECT
      ///ATRIBUTOS GENERALES
      map<JointType,string> jointNames; //TABLA ENUM JOINT CON SUS RESPECTIVOS STRING
      map<JointType,string>::iterator jointNamesIt;
      int IMAGE_WIDTH, IMAGE_HEIGHT;
      
      ///ATRIBUTOS NECESARIOS PARA LA LECTURA DE FLUJOS
      openni::Status openniRc;
      Device device;
      VideoStream depth;
      VideoStream color;
      VideoFrameRef depthFrame;
      VideoFrameRef colorFrame;
      
      ///ATRIBUTOS NECESARIOS PARA EL TRACKINGN DE NiTE
      bool g_visibleUsers[MAX_USERS];	//VECTOR DE USUARIOS VISIBLES
      SkeletonState g_skeletonStates[MAX_USERS]; //VECTOR DE ESTADOS DE LOS USUARIOS
      UserTracker userTracker;
      nite::Status niteRc;
      UserTrackerFrameRef userTrackerFrame;   
      
      ///ATRIBUTOS PARA LA OBTENCIÓN DE COLOR Y PROFUNDIDAD CON OPENNI
      VideoStream* pStream;
      int changedStreamDummy;
      DepthPixel* pixDepth;
      RGB888Pixel* pixColor;
      RoboCompRGBD::DepthSeq* depthBuffer;
      RoboCompRGBD::ColorSeq* colorBuffer;
      imgType* colorImage;
      depthType* depthImage;

      ///ATRIBUTOS PARA LA GESTIÓN DE TRACKING DE USUARIOS DE NiTE
      SkeletonJoint  bodyJoint; //PARTE DEL CUERPO
      joint position;	//POSICION PARA PARTE DEL CUERPO
      RTMatrix rotation;
      TPerson person;
      PersonList trackedUsers; //IDENTIFICADOR DE USUARIO + TPerson
      PersonList finalTrackedUsers;
      map<string,RTMat> mapJointRotations;
 
      ///MUTEX
      QMutex *usersMutex, *RGBMutex, *depthMutex;
      
      ///ATRIBUTOS PARA PINTAR       
      vector<short> normalDepth;
      RCDraw *drawRGB,*drawDepth;
      Ipp16u *mColor;
      Ipp8u *auxDepth;
      IppiSize ippSizeImage;      
      QImage *qImgDepth;
      CoordinateConverter conversor;

      Registration registration;
      
      ///ATRIBUTOS PARA GRABACIÓN
      Recorder recorder;
      time_t startTime;
      time_t currentTime;
      CapturingState capturingState;
      
      ///MÉTODOS PRIVADOS
      void openDevice();
      bool openStream(SensorType sensorType, VideoStream *stream);
      void initializeStreams();
      void initializeTracking();
      void readFrame();
      void readColor();
      void readDepth();
      void updateUsersData();
      
      void calculateJointRotations(TPerson &p);
      RTMat rtMatFromJointPosition(RTMat rS, joint p1, joint p2, joint translation, int axis);
      bool rotarTorso(const QVec & hombroizq,const QVec & hombroder);
      void setPersonRotations(TPerson &p);
           
      void updateUserState(const nite::UserData& user, unsigned long long ts);
      void paintDepth();
      void normalizeDepth();
      void drawLimb(short int id, string eJoint1, string eJoint2);
      void drawBody(short int id);      
      
      void startRecorder();
      void runRecorder();
      void stopRecorder();
private slots:
      void clickStartButton();
      void clickStopButton();
      
public:
	SpecificWorker(MapPrx& mprx, QObject *parent = 0);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	
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

	///HUMAN TRACKER INTERFACE
	void  getJointsPosition(int id, jointListType& jointList);	//Devuelve lista de joints
 	void  getRTMatrixList(int id, RTMatrixList& RTMatList);		//Devuelve lista de rotaciones
	void  getUserState(int id, TrackingState &state);		//Devuelve estado de un usuario
	void  getUser(int id, TPerson &user);				//Devuelve un usuario
	void  getUsersList(PersonList &users);		//Devuelve una lista de usuarios

	
 	

public slots:
 	void compute(); 	
};

#endif

