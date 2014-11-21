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

/**
* \brief Default constructor
*/

SpecificWorker::SpecificWorker(MapPrx& mprx, QObject *parent) : GenericWorker(mprx, parent)
{
	jointNames[JOINT_HEAD]="JOINT_HEAD"; jointNames[JOINT_NECK]="JOINT_NECK"; jointNames[JOINT_LEFT_SHOULDER]="JOINT_LEFT_SHOULDER";
	jointNames[JOINT_RIGHT_SHOULDER]="JOINT_RIGHT_SHOULDER"; jointNames[JOINT_LEFT_ELBOW]="JOINT_LEFT_ELBOW";
	jointNames[JOINT_RIGHT_ELBOW]="JOINT_RIGHT_ELBOW"; jointNames[JOINT_LEFT_HAND]="JOINT_LEFT_HAND";
	jointNames[JOINT_RIGHT_HAND]="JOINT_RIGHT_HAND"; jointNames[JOINT_TORSO]="JOINT_TORSO"; jointNames[JOINT_LEFT_HIP]="JOINT_LEFT_HIP";
	jointNames[JOINT_RIGHT_HIP]="JOINT_RIGHT_HIP"; jointNames[JOINT_LEFT_KNEE]="JOINT_LEFT_KNEE";
	jointNames[JOINT_RIGHT_KNEE]="JOINT_RIGHT_KNEE"; jointNames[JOINT_LEFT_FOOT]="JOINT_LEFT_FOOT"; jointNames[JOINT_RIGHT_FOOT]="JOINT_RIGHT_FOOT";

	openDevice();
	initializeStreams();
	initializeTracking();

	///INICIALIZACION ATRIBUTOS GENERALES
	registration=RoboCompRGBD::None; //A QUE INICIALIZO REGISTRATION????
	capturingState=NOT_CAPTURING;

	///INICIALIZACION MUTEX
	usersMutex = new QMutex(); RGBMutex = new QMutex(); depthMutex = new QMutex();

	///INICIALIZACION ATRIBUTOS PARA PINTAR
	ippSizeImage.width = IMAGE_WIDTH; ippSizeImage.height = IMAGE_HEIGHT;
	mColor = ippsMalloc_16u (IMAGE_WIDTH*IMAGE_HEIGHT);
	auxDepth = ippsMalloc_8u (IMAGE_WIDTH*IMAGE_HEIGHT);
	qImgDepth = new QImage(IMAGE_WIDTH,IMAGE_HEIGHT,QImage::Format_Indexed8);
	drawDepth = new RCDraw (IMAGE_WIDTH,IMAGE_HEIGHT,qImgDepth, this->frameDepth);
	connect(startButton,SIGNAL(released()),this,SLOT(clickStartButton()));
	connect(stopButton,SIGNAL(released()),this,SLOT(clickStopButton()));
	printf("\nStart moving around to get detected...\n(PSI pose may be required for skeleton calibration, depending on the configuration)\n");
	setPeriod(33);
}

void SpecificWorker::openDevice()
{
	openniRc = OpenNI::initialize();
	if (openniRc != openni::STATUS_OK)
	{
		OpenNI::shutdown();
		qFatal("openNi2Comp: OpenNI initialize failed: \n%s\n", OpenNI::getExtendedError());
	}

	openniRc = device.open(ANY_DEVICE);
		//   openniRc = device.open("/home/robocomp/robocomp/components/programaposturas/Prueba.oni");
	if (openniRc != openni::STATUS_OK)
	{
		OpenNI::shutdown();
		qFatal("openNi2Comp: Device open failed: \n%s\n", OpenNI::getExtendedError());
	}

	if(device.isFile())
	{
		qDebug("Es un archivo");
		startButton->setEnabled(false);
	}
	else
	{
		qDebug("Es un dispositivo fisico");
	}
}

void SpecificWorker::initializeStreams()
{
	if (!openStream(SENSOR_DEPTH, &depth)) qFatal("OPEN DEPTH STREAM FAILED!");
	if (!openStream(SENSOR_COLOR, &color)) qFatal("OPEN COLOR STREAM FAILED!");

	IMAGE_WIDTH=depth.getVideoMode().getResolutionX();
	IMAGE_HEIGHT=depth.getVideoMode().getResolutionY();

	depthBuffer = new vector<float>(IMAGE_WIDTH*IMAGE_HEIGHT);
	colorBuffer = new vector<ColorRGB>(IMAGE_WIDTH*IMAGE_HEIGHT);
	depthImage = new vector<float>(IMAGE_WIDTH*IMAGE_HEIGHT);
	colorImage = new vector<Ice::Byte>(IMAGE_WIDTH*IMAGE_HEIGHT*3); //x3 para RGB888Pixel
}

bool SpecificWorker::openStream(SensorType sensorType, VideoStream *stream)
{
	openniRc = stream->create(device, sensorType);
	if (openniRc != openni::STATUS_OK)
	{
		printf("openNi2Comp: Couldn't find stream:\n%s\n", OpenNI::getExtendedError());
		return false;
	}
	openniRc = stream->start();
	if (openniRc != openni::STATUS_OK)
	{
		printf("openNi2Comp: Couldn't start stream:\n%s\n", OpenNI::getExtendedError());
		stream->destroy();
		return false;
	}
	if (!stream->isValid())
		return false;

	return true;
}

void SpecificWorker::initializeTracking()
{
	g_visibleUsers[0] = false;
	g_skeletonStates[0] = SKELETON_NONE;
	NiTE::initialize();
	niteRc = userTracker.create(&device);
	if (niteRc != nite::STATUS_OK)
		qFatal("Couldn't create user tracker\n");

	int i;
	for (i=0;i<TAM_JOINT;i++) position.push_back(0);
	for (i=0;i<TAM_ROTATION;i++) rotation.push_back(0);
	person.state=RoboCompHumanTracker::None;
	for (jointNamesIt=jointNames.begin(); jointNamesIt != jointNames.end(); jointNamesIt++)
	{
		person.joints[jointNamesIt->second]=position;
		person.rotations[jointNamesIt->second]=rotation;
	}
	person.joints["JOINT_WAIST"]=position;
	person.rotations["JOINT_WAIST"]=rotation;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	ippFree(mColor);
	ippFree(auxDepth);
	depth.stop(); depth.destroy();
	color.stop(); color.destroy();
	device.close();
	NiTE::shutdown(); OpenNI::shutdown();
	delete depthBuffer; delete colorBuffer;
	delete depthImage; delete colorImage;
	delete qImgDepth; delete drawDepth;
	delete usersMutex; delete RGBMutex; delete depthMutex;
}


void SpecificWorker::compute( )
{
      readFrame();
      runRecorder();
      paintDepth();
      updateUsersData();

      drawDepth->update();
}

void SpecificWorker::readFrame()
{
	openni::Status rc = openni::STATUS_OK;

	openni::VideoStream* streams[] = {&depth, &color};

	int changedIndex = -1;
	while (rc == openni::STATUS_OK)
	{
		rc = openni::OpenNI::waitForAnyStream(streams, 2, &changedIndex, 0);
		if (rc == openni::STATUS_OK)
		{
			switch (changedIndex)
			{
			case 0:
				readDepth(); break;
			case 1:
				readColor(); break;
			default:
				printf("Error in wait\n");
			}
		}
	}
}

void SpecificWorker::readColor()
{
	openniRc = color.readFrame(&colorFrame);
	if (openniRc != openni::STATUS_OK)
	{
		printf("Read color failed!\n%s\n", OpenNI::getExtendedError());
		return;
	}

	if (colorFrame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_RGB888)
	{
		printf("Unexpected color frame format\n");
		return;
	}

	pixColor = (RGB888Pixel*)colorFrame.getData();

	RGBMutex->lock();
	memcpy(&colorImage->operator[](0),&pixColor[0],IMAGE_HEIGHT*IMAGE_WIDTH*sizeof(RGB888Pixel));
	RGBMutex->unlock();
}


void SpecificWorker::readDepth()
{
	openniRc = depth.readFrame(&depthFrame);
	if (openniRc != openni::STATUS_OK)
	{
		printf("Read depth failed!\n%s\n", OpenNI::getExtendedError());
		return;
	}

	if (depthFrame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_1_MM && depthFrame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_100_UM)
	{
		printf("Unexpected depth frame format\n");
		return;
	}

	pixDepth = (DepthPixel*)depthFrame.getData();
	///¿PUEDE HACERSE CON MEMCPY? DISTINTOS TIPOS... (short, float). Creo que NO
	//memcpy(&depthBuffer->operator[](0),pixDepth,IMAGE_WIDTH*IMAGE_HEIGHT*sizeof(float));	//ESTO SUSTITUIRÍA AL FOR DE DEBAJO
	for(int i=0;i<IMAGE_WIDTH*IMAGE_HEIGHT;i++) depthBuffer->operator[](i)=pixDepth[i];
	depthMutex->lock();
	depthImage=depthBuffer;
	//memcpy(&depthImage->operator[](0),&depthBuffer->operator[](0),IMAGE_WIDTH*IMAGE_HEIGHT*sizeof(float)); //ESTO SUSTITUIRÍA AL = DE ARRIBA
	depthMutex->unlock();
}

void SpecificWorker::updateUsersData()
{
	niteRc = userTracker.readFrame(&userTrackerFrame);
	if (niteRc != nite::STATUS_OK){
		printf("Get next frame failed\n");
		return;
	}
	const nite::Array<UserData>& users = userTrackerFrame.getUsers();
	trackedUsers.clear();
	for (int i = 0; i < users.getSize(); ++i)
	{
		const UserData& user = users[i];
		updateUserState(user,userTrackerFrame.getTimestamp());
		if (user.isNew())
		{
			userTracker.startSkeletonTracking(user.getId());
			trackedUsers[user.getId()]=person;
		}
		else /*if (user.getSkeleton().getState() == SKELETON_TRACKED)*/
		{
			for(jointNamesIt=jointNames.begin(); jointNamesIt != jointNames.end(); jointNamesIt++)
			{
				bodyJoint= user.getSkeleton().getJoint(jointNamesIt->first);
				position[0]=bodyJoint.getPosition().x; position[1]=bodyJoint.getPosition().y;
				position[2]=bodyJoint.getPosition().z; position[3]=bodyJoint.getPositionConfidence();
				person.joints[jointNamesIt->second]=position;
			}
			///CALCULAMOS CINTURA
			person.joints["JOINT_WAIST"][0]=((person.joints["JOINT_LEFT_HIP"][0]+person.joints["JOINT_RIGHT_HIP"][0]+person.joints["JOINT_TORSO"][0]))/3;
			person.joints["JOINT_WAIST"][1]=((person.joints["JOINT_LEFT_HIP"][1]+person.joints["JOINT_RIGHT_HIP"][1]+person.joints["JOINT_TORSO"][1]))/3;
			person.joints["JOINT_WAIST"][2]=((person.joints["JOINT_LEFT_HIP"][2]+person.joints["JOINT_RIGHT_HIP"][2]+person.joints["JOINT_TORSO"][2]))/3;
			person.joints["JOINT_WAIST"][3]=((person.joints["JOINT_LEFT_HIP"][3]+person.joints["JOINT_RIGHT_HIP"][3]+person.joints["JOINT_TORSO"][3]))/3;
			//qDebug() << person.joints["JOINT_WAIST"][0] << person.joints["JOINT_WAIST"][1] << person.joints["JOINT_WAIST"][2] << person.joints["JOINT_WAIST"][3];
			calculateJointRotations(person);
			setPersonRotations(person);	//Pasa las rotaciones de la estructura de datos del cálculo, a la estructura de datos de la i¡nterfaz
			trackedUsers[user.getId()]=person;
			drawBody(users[i].getId());
		}
	}
	usersMutex->lock();
	finalTrackedUsers=trackedUsers;
	usersMutex->unlock();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
}

void SpecificWorker::updateUserState(const nite::UserData& user, unsigned long long ts)
{
	if (user.isNew())
		USER_MESSAGE("New")
	else if (user.isVisible() && !g_visibleUsers[user.getId()])
		USER_MESSAGE("Visible")
	else if (!user.isVisible() && g_visibleUsers[user.getId()])
		USER_MESSAGE("Out of Scene")
	else if (user.isLost())
		USER_MESSAGE("Lost")

	g_visibleUsers[user.getId()] = user.isVisible();

	if(g_skeletonStates[user.getId()] != user.getSkeleton().getState())
	{
		switch(g_skeletonStates[user.getId()] = user.getSkeleton().getState())
		{
		case SKELETON_NONE:
			USER_MESSAGE("Stopped tracking.")
			person.state = RoboCompHumanTracker::None;
			break;
		case SKELETON_CALIBRATING:
			USER_MESSAGE("Calibrating...")
			person.state = RoboCompHumanTracker::Calibrating;
			break;
		case SKELETON_TRACKED:
			USER_MESSAGE("Tracking!")
			person.state = RoboCompHumanTracker::Tracked;
			break;
		case SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
			person.state = RoboCompHumanTracker::ErrorNotInPose;
			break;
		case SKELETON_CALIBRATION_ERROR_HANDS:
			person.state = RoboCompHumanTracker::ErrorHands;
			break;
		case SKELETON_CALIBRATION_ERROR_LEGS:
			person.state = RoboCompHumanTracker::ErrorLegs;
			break;
		case SKELETON_CALIBRATION_ERROR_HEAD:
			person.state = RoboCompHumanTracker::ErrorHead;
			break;
		case SKELETON_CALIBRATION_ERROR_TORSO:
			USER_MESSAGE("Calibration Failed... :-|")
			person.state = RoboCompHumanTracker::ErrorTorso;
			break;
		}
	}
}


void SpecificWorker::paintDepth()
{
	memcpy(mColor,(DepthPixel*)depthFrame.getData(),depthFrame.getDataSize());
	//a centimetros, para no desbordar los enteros
	ippiDivC_16u_C1IRSfs((Ipp16u)100,mColor,IMAGE_WIDTH*sizeof(Ipp16u),ippSizeImage,0);
	//mapeo a 255 colores, escala de grises.
	ippiMulC_16u_C1IRSfs((Ipp16u)(255/(MAX_DEPTH/100)),mColor,IMAGE_WIDTH*sizeof(Ipp16u),ippSizeImage,0);
	//a uchar
	ippiConvert_16u8u_C1R(mColor,IMAGE_WIDTH*sizeof(Ipp16u),auxDepth,IMAGE_WIDTH*sizeof(Ipp8u),ippSizeImage);

	memcpy(qImgDepth->bits(),auxDepth,IMAGE_WIDTH*IMAGE_HEIGHT);
}

void SpecificWorker::normalizeDepth()
{
	for (int i=0; i<(IMAGE_HEIGHT*IMAGE_WIDTH); i++)
	{
		normalDepth[i]=255-(255*depthBuffer->operator[](i)/5/1000);
		if (normalDepth[i] > 255) normalDepth[i] = 255;
		if (normalDepth[i] < 0) normalDepth[i] = 0;
	}
}

void SpecificWorker::drawLimb(short int id, string eJoint1, string eJoint2)
{
	if (trackedUsers[id].joints[eJoint1][3] < 0.5 || trackedUsers[id].joints[eJoint2][3] < 0.5) return;

	vector<float> pt1 = trackedUsers[id].joints[eJoint1];
	vector<float> pt2 = trackedUsers[id].joints[eJoint2];
	userTracker.convertJointCoordinatesToDepth(pt1[0], pt1[1], pt1[2], &pt1[0], &pt1[1]);
	userTracker.convertJointCoordinatesToDepth(pt2[0], pt2[1], pt2[2], &pt2[0], &pt2[1]);
	//LO SIGUIENTE ES OTRA FORMA DE CONVERTIR LOS PUNTOS AL PLANO QUE TAMBIÉN FUNCIONA, PERO ME PARECE MÁS INTUITIVA LA QUE USO
	//conversor.convertWorldToDepth(depth, (float)pt[0].x,(float)pt[0].y,(float)pt[0].z, &pt[0].x,&pt[0].y, pixDepth);
	//conversor.convertWorldToDepth(depth, (float)pt[1].x,(float)pt[1].y,(float)pt[1].z, &pt[1].x,&pt[1].y, pixDepth);
	drawDepth->drawLineOnTop (QLine(pt1[0],pt1[1],pt2[0],pt2[1]),Qt::green,4);
}

void SpecificWorker::drawBody(short int id)
{
	if(trackedUsers.find(id) == trackedUsers.end()) return;

	drawLimb(id,"JOINT_HEAD","JOINT_NECK");
	drawLimb(id,"JOINT_NECK","JOINT_LEFT_SHOULDER");
	drawLimb(id,"JOINT_NECK","JOINT_RIGHT_SHOULDER");
	drawLimb(id,"JOINT_LEFT_SHOULDER","JOINT_RIGHT_SHOULDER");
	drawLimb(id,"JOINT_LEFT_SHOULDER","JOINT_TORSO");
	drawLimb(id,"JOINT_RIGHT_SHOULDER","JOINT_TORSO");
	drawLimb(id,"JOINT_TORSO","JOINT_LEFT_HIP");
	drawLimb(id,"JOINT_TORSO","JOINT_RIGHT_HIP");
	drawLimb(id,"JOINT_LEFT_HIP","JOINT_RIGHT_HIP");
	drawLimb(id,"JOINT_LEFT_SHOULDER","JOINT_LEFT_ELBOW");
	drawLimb(id,"JOINT_RIGHT_SHOULDER","JOINT_RIGHT_ELBOW");
	drawLimb(id,"JOINT_LEFT_ELBOW","JOINT_LEFT_HAND");
	drawLimb(id,"JOINT_RIGHT_ELBOW","JOINT_RIGHT_HAND");
	drawLimb(id,"JOINT_LEFT_HIP","JOINT_LEFT_KNEE");
	drawLimb(id,"JOINT_RIGHT_HIP","JOINT_RIGHT_KNEE");
	drawLimb(id,"JOINT_LEFT_KNEE","JOINT_LEFT_FOOT");
	drawLimb(id,"JOINT_RIGHT_KNEE","JOINT_RIGHT_FOOT");
}



TRGBDParams SpecificWorker::getRGBDParams( )
{
	//¿CAMBIA ALGO DE ESTO DE UN FRAME A OTRO?
	TRGBDParams params;
	//params.color.focal=;
	params.color.width=IMAGE_WIDTH;
	params.color.height=IMAGE_HEIGHT;
	params.color.size=IMAGE_WIDTH*IMAGE_HEIGHT*3;
	//params.color.FPS=;
	//params.depth.focal=;
	params.depth.width=IMAGE_WIDTH;
	params.depth.height=IMAGE_HEIGHT;
	params.depth.size=IMAGE_WIDTH*IMAGE_HEIGHT;
	//params.depth.FPS=;¡
	params.timerPeriod=Period;
	//params.talkToBase=;
	//params.talkToJointMotor=;
	//params.driver=;
	//params.device=;
	return params;
}

void SpecificWorker::setRegistration (RoboCompRGBD::Registration value)
{
	registration=value;
}

Registration SpecificWorker::getRegistration ( ){
	return registration;
}

void SpecificWorker::getData(imgType& rgbMatrix, depthType& distanceMatrix, RoboCompJointMotor::MotorStateMap& hState, RoboCompDifferentialRobot::TBaseState& bState)
{
	RGBMutex->lock();
	rgbMatrix=*colorImage;
	RGBMutex->unlock();
	depthMutex->lock();
	distanceMatrix=*depthImage;
	depthMutex->unlock();

}

void SpecificWorker::getDepthInIR(depthType& distanceMatrix, RoboCompJointMotor::MotorStateMap &hState, RoboCompDifferentialRobot::TBaseState& bState)
{
	qDebug()<<"getDepthInIR not implemented yet";
}

void SpecificWorker::getImage(ColorSeq& color, DepthSeq& depth, PointSeq& points, RoboCompJointMotor::MotorStateMap &hState, RoboCompDifferentialRobot::TBaseState& bState)
{
	getDepth(depth,hState,bState);
	getRGB(color,hState,bState);
	getXYZ(points,hState,bState);
}

void SpecificWorker::getDepth(DepthSeq& depth, RoboCompJointMotor::MotorStateMap &hState, RoboCompDifferentialRobot::TBaseState& bState )
{
	depthMutex->lock();
	depth=*depthBuffer;
	depthMutex->unlock();
}

void SpecificWorker::getRGB(ColorSeq& color, RoboCompJointMotor::MotorStateMap &hState, RoboCompDifferentialRobot::TBaseState& bState)
{
	RGBMutex->lock();
	color=*colorBuffer;
	RGBMutex->unlock();
}

void SpecificWorker::getXYZ(PointSeq& points, RoboCompJointMotor::MotorStateMap &hState, RoboCompDifferentialRobot::TBaseState& bState)
{
	qDebug()<<"getXYZ not implemented yet";
}

void  SpecificWorker::getJointsPosition(int id, jointListType& jointList)
{
	usersMutex->lock();
	jointList=finalTrackedUsers[id].joints;
	usersMutex->unlock();
}

void  SpecificWorker::getRTMatrixList(int id, RTMatrixList& RTMatList)
{
	usersMutex->lock();
	RTMatList=finalTrackedUsers[id].rotations;
	usersMutex->unlock();
}

void  SpecificWorker::getUserState(int id, TrackingState &state)
{
	usersMutex->lock();
	state=finalTrackedUsers[id].state;
	usersMutex->unlock();
}

void SpecificWorker::getUser(int id, TPerson &user)
{
	usersMutex->lock();
	user=finalTrackedUsers[id];
	usersMutex->unlock();
}

void SpecificWorker::getUsersList(PersonList &users)
{
	usersMutex->lock();
	users=finalTrackedUsers;
	usersMutex->unlock();
}

void SpecificWorker::calculateJointRotations(TPerson &p)
{
	RTMat kinect;

	/// apunta el torso (inclinación alante/atrás y lateral del torso)
	mapJointRotations["JOINT_TORSO"]=rtMatFromJointPosition (kinect,p.joints["JOINT_TORSO"],p.joints["JOINT_NECK"], p.joints["JOINT_TORSO"], 2);

	/// alineación de hombros (rotación en Z del torso), previa al cálculo de la transformacion final de los hombros.
	RTMat LEFT_SHOULDER_PRE_Z=rtMatFromJointPosition(mapJointRotations["JOINT_TORSO"],p.joints["JOINT_LEFT_SHOULDER"],p.joints["JOINT_LEFT_ELBOW"], p.joints["JOINT_LEFT_SHOULDER"], 2);
	RTMat RIGHT_SHOULDER_PRE_Z=rtMatFromJointPosition(mapJointRotations["JOINT_TORSO"],p.joints["JOINT_RIGHT_SHOULDER"],p.joints["JOINT_RIGHT_ELBOW"],p.joints["JOINT_RIGHT_SHOULDER"], 2);

	rotarTorso (RIGHT_SHOULDER_PRE_Z.getTr(), LEFT_SHOULDER_PRE_Z.getTr());

	///Cintura
	mapJointRotations["JOINT_WAIST"]=rtMatFromJointPosition (mapJointRotations["JOINT_TORSO"],p.joints["JOINT_WAIST"],p.joints["JOINT_TORSO"] ,p.joints["JOINT_WAIST"], 2);
	mapJointRotations["JOINT_LEFT_HIP"]=rtMatFromJointPosition (mapJointRotations["JOINT_TORSO"],p.joints["JOINT_LEFT_HIP"],p.joints["JOINT_LEFT_KNEE"],p.joints["JOINT_LEFT_HIP"], 2);
	mapJointRotations["JOINT_RIGHT_HIP"]=rtMatFromJointPosition (mapJointRotations["JOINT_TORSO"],p.joints["JOINT_RIGHT_HIP"],p.joints["JOINT_RIGHT_KNEE"],p.joints["JOINT_RIGHT_HIP"], 2);

	///Kneee
	mapJointRotations["JOINT_LEFT_KNEE"]=rtMatFromJointPosition (mapJointRotations["JOINT_TORSO"]*mapJointRotations["JOINT_LEFT_HIP"],p.joints["JOINT_LEFT_KNEE"],p.joints["JOINT_LEFT_FOOT"],p.joints["JOINT_LEFT_KNEE"], 2);

	mapJointRotations["JOINT_RIGHT_KNEE"]=rtMatFromJointPosition (mapJointRotations["JOINT_TORSO"]*mapJointRotations["JOINT_RIGHT_HIP"],p.joints["JOINT_RIGHT_KNEE"],p.joints["JOINT_RIGHT_FOOT"],p.joints["JOINT_RIGHT_KNEE"], 2);

	///cabeza
	mapJointRotations["JOINT_NECK"]=rtMatFromJointPosition (mapJointRotations["JOINT_TORSO"],p.joints["JOINT_NECK"],p.joints["JOINT_HEAD"],p.joints["JOINT_NECK"], 2);

	///brazo izquierdo
	mapJointRotations["JOINT_LEFT_SHOULDER"]=rtMatFromJointPosition (mapJointRotations["JOINT_TORSO"],p.joints["JOINT_LEFT_SHOULDER"],p.joints["JOINT_LEFT_ELBOW"],p.joints["JOINT_LEFT_SHOULDER"], 2);

	mapJointRotations["JOINT_LEFT_ELBOW"]=rtMatFromJointPosition (mapJointRotations["JOINT_TORSO"]*mapJointRotations["JOINT_LEFT_SHOULDER"],p.joints["JOINT_LEFT_ELBOW"],p.joints["JOINT_LEFT_HAND"],	p.joints["JOINT_LEFT_ELBOW"], 2);


	///brazo derecho
	///codo al hombro (p2 inicio p1 final)
	mapJointRotations["JOINT_RIGHT_SHOULDER"]=rtMatFromJointPosition(mapJointRotations["JOINT_TORSO"],p.joints["JOINT_RIGHT_SHOULDER"], p.joints["JOINT_RIGHT_ELBOW"],p.joints["JOINT_RIGHT_SHOULDER"], 2);

	///
	mapJointRotations["JOINT_RIGHT_ELBOW"]=rtMatFromJointPosition (mapJointRotations["JOINT_TORSO"]*mapJointRotations["JOINT_RIGHT_SHOULDER"],p.joints["JOINT_RIGHT_ELBOW"],p.joints["JOINT_RIGHT_HAND"],p.joints["JOINT_RIGHT_ELBOW"], 2);

	///---------------------------------------------------------------------------
	mapJointRotations["JOINT_RIGHT_HAND"]=rtMatFromJointPosition(mapJointRotations["JOINT_TORSO"]*mapJointRotations["JOINT_RIGHT_SHOULDER"]*mapJointRotations["JOINT_RIGHT_ELBOW"],p.joints["JOINT_RIGHT_HAND"],p.joints["JOINT_RIGHT_ELBOW"],p.joints["JOINT_RIGHT_HAND"], 2);
	mapJointRotations["JOINT_LEFT_HAND"]=rtMatFromJointPosition(mapJointRotations["JOINT_TORSO"]*mapJointRotations["JOINT_LEFT_SHOULDER"]*mapJointRotations["JOINT_LEFT_ELBOW"],p.joints["JOINT_LEFT_HAND"],p.joints["JOINT_LEFT_ELBOW"],p.joints["JOINT_LEFT_HAND"], 2);

	mapJointRotations["JOINT_RIGHT_FOOT"]=rtMatFromJointPosition(mapJointRotations["JOINT_TORSO"]*mapJointRotations["JOINT_RIGHT_HIP"]*mapJointRotations["JOINT_RIGHT_KNEE"],p.joints["JOINT_RIGHT_FOOT"],p.joints["JOINT_RIGHT_KNEE"],p.joints["JOINT_RIGHT_FOOT"], 2);
	mapJointRotations["JOINT_LEFT_FOOT"]=rtMatFromJointPosition(mapJointRotations["JOINT_TORSO"]*mapJointRotations["JOINT_LEFT_HIP"]*mapJointRotations["JOINT_LEFT_KNEE"],p.joints["JOINT_LEFT_FOOT"],p.joints["JOINT_LEFT_KNEE"],p.joints["JOINT_LEFT_FOOT"], 2);

}

RTMat SpecificWorker::rtMatFromJointPosition(RTMat rS, joint p1, joint p2, joint translation, int axis)
{
	bool XClockWise=true, YClockWise=true, ZClockWise=true;
	float alpha, beta, gamma;

	RTMat rt(XClockWise,YClockWise, ZClockWise);
	QVec p1h = QVec::vec4(p1,1);
	QVec p2h = QVec::vec4(p2,1);
	QVec translationH = QVec::vec4(translation,1);

	QMat aux =rS;
	aux = aux.invert();
	QVec translationT = aux *translationH;
	QVec p1t = aux * p1h;
	QVec p2t = aux * p2h;
	QVec v = p2t - p1t;

	v= v.normalize();

	///por filas
	switch(axis)
	{
	case 0:
		alpha = 0;
		if (YClockWise) beta = atan2(-v.z(),v.x());
		else beta = atan2(v.z(),v.x());
		if (ZClockWise) gamma = asin(-v.y());
		else gamma = asin(v.y());
		break;
	case 1:
		if (XClockWise) alpha = atan2(v.z(),v.y());
		else alpha = atan2(-v.z(),v.y());
		beta = 0;
		if (ZClockWise) gamma = asin(v.x());
		else gamma = asin(-v.x());
		break;
	case 2:
		if (XClockWise) alpha =  atan2(-v.y(),v.z());
		else alpha =  atan2(v.y(),v.z());
		if (YClockWise) beta = asin(v.x());
		else beta = asin(-v.x());
		gamma = 0;
		break;
	}
	rt.setRT(alpha,beta,gamma,translationT);
	return rt;
}

bool SpecificWorker::rotarTorso(const QVec & hombroizq, const QVec & hombroder)
{
    QVec eje= hombroizq - hombroder;	//Calculamos el eje que va de un hombro a otro
    eje.normalize();

    if(eje.x()==0) return false;

    float angulo = atan2(eje.y(),eje.x());	//Calculamos el giro necesario para alinear los hombros con el eje
    mapJointRotations["JOINT_TORSO"].setRZ(angulo); // Aplicamos dicho giro al eje Z del torso
    return true;
}

void SpecificWorker::setPersonRotations(TPerson &p)
{
	for(jointNamesIt=jointNames.begin(); jointNamesIt != jointNames.end(); jointNamesIt++)
	{
		p.rotations[jointNamesIt->second][0]=mapJointRotations[jointNamesIt->second].getRxValue();
		p.rotations[jointNamesIt->second][1]=mapJointRotations[jointNamesIt->second].getRyValue();
		p.rotations[jointNamesIt->second][2]=mapJointRotations[jointNamesIt->second].getRzValue();
		p.rotations[jointNamesIt->second][3]=mapJointRotations[jointNamesIt->second].getTr().x();
		p.rotations[jointNamesIt->second][4]=mapJointRotations[jointNamesIt->second].getTr().y();
		p.rotations[jointNamesIt->second][5]=mapJointRotations[jointNamesIt->second].getTr().z();
	}
}

void SpecificWorker::startRecorder()
{
	openni::Status recorderStatus;
	recorderStatus=recorder.create("Video.oni");
	if (recorderStatus != openni::STATUS_OK)
	{
		recorder.destroy();
		qDebug("Create failed");
		return;
	}

	time(&startTime);
	capturingState=SHOULD_CAPTURE;
	qDebug("Record should start...");
}

void SpecificWorker::runRecorder()
{
	switch(capturingState)
	{
	case NOT_CAPTURING:
		break;
	case CAPTURING:
		time(&currentTime);
		secCount->display(difftime(currentTime,startTime));
		break;
	case SHOULD_CAPTURE:
		openni::Status recorderStatus, recorderStatus2;
		recorderStatus= recorder.attach(depth);
		recorderStatus2= recorder.attach(color,1);
		if (recorderStatus != openni::STATUS_OK || recorderStatus2 != openni::STATUS_OK)
		{
			recorder.destroy();
			qDebug("Atach failed");
			return;
		}
		recorderStatus= recorder.start();
		if (recorderStatus != openni::STATUS_OK)
		{
			recorder.destroy();
			qDebug("Start failed");
			return;
		}
		qDebug("Record started...");
		capturingState = CAPTURING;
		break;
	}
}

void SpecificWorker::stopRecorder()
{
	if(recorder.isValid())
	{
		recorder.stop();
		recorder.destroy();
		capturingState = NOT_CAPTURING;
		qDebug("Record finished...");
	}
}

void SpecificWorker::clickStartButton()
{
	startRecorder();
	stopButton->setEnabled(true);
	startButton->setEnabled(false);
}

void SpecificWorker::clickStopButton()
{
	stopRecorder();
	startButton->setEnabled(true);
	stopButton->setEnabled(false);
}
