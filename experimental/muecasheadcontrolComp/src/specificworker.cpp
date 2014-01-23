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

SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)	
{
		qDebug()<<"init worker";
	mutex = new QMutex(QMutex::Recursive);
	mutex_memory = new QMutex(QMutex::Recursive);
	
		//leer del config	&& pasar a una clase la lectura de parametros del config
	//basic period leerlo de algun sitio o pasarlo por parametros
	neckMotorName ="neck";
	tiltMotorName ="tilt";
	leftPanMotorName = "leftPan";
	rightPanMotorName = "rightPan";
	
	mouthMotorName= "mouth";
	eyeBLeftUPMotorName = "eyeBLeftUP";
	eyeBLeftYAWMotorName= "eyeBLeftYAW";
	eyeBRightUPMotorName= "eyeBRightUP";
	eyeBRightYAWMotorName = "eyeBRightYAW";
	
	/////////////////camera ///////////////////////////////////
	
	try
	{
		paramsCamera = camera_proxy->getCamParams();
	}
	catch ( const Ice::Exception& ex )
	{
		qDebug ( " Could not obtain camera parameters... \n" );
		std::cout << ex<<std::endl;
		exit(-1);
		
	}
		imgSize = paramsCamera.width * paramsCamera.height ;
	
	qDebug() << "Focal: " << paramsCamera.focal;
	qDebug() << "Width " <<  paramsCamera.width;
	qDebug() << "Height " << paramsCamera.height;
	
	
	////////////////////faulhaber y motor///////////////////////////////
	
		//read params from faulhaber and muecasJoint
	try{
		mplFaulhaber = jointmotor1_proxy->getAllMotorParams();
		mplAll = mplFaulhaber;
	}
	catch(Ice::Exception &e)
	{
		std::cout << e<<std::endl;
	}
	try{		
		mplMuecas =jointmotor0_proxy->getAllMotorParams();
		foreach (RoboCompJointMotor::MotorParams motor, mplMuecas)
			mplAll.push_back(motor);
	}
	catch(Ice::Exception &e)
	{
		std::cout << e<<std::endl;
	}
	//Now checking for motors in JointMotor that match localParams
	int cont=0;
	QVector<RoboCompJointMotor::MotorParams> qmp = QVector<RoboCompJointMotor::MotorParams>::fromStdVector( mplFaulhaber );
	foreach( RoboCompJointMotor::MotorParams qmp, mplFaulhaber)
	{
		if (qmp.name == neckMotorName  or  qmp.name == tiltMotorName  or
			qmp.name == leftPanMotorName or  qmp.name == rightPanMotorName )
		{
			headParams.motorsParams[qmp.name] = qmp;
			cont++;
		}
		else
			qDebug() << "muecasHead::Monitor::Initialize() - No required motor found in running JointMotor: "<<qmp.name.c_str();
	}
	if(cont != 4)
	{
		qDebug() << "muecasHead::Monitor::Initialize() - Required motors not found in running JointMotor";
		qFatal("error initializing head motors");
	}
	headParams.model = "NT2P";

	qDebug() << "muecasHead() - constructor Ok";
	
	////////////////////////////////////////////////////////////////////////////
	
	
	qImageRGB = new QImage(paramsCamera.width,paramsCamera.height, QImage::Format_Indexed8);
	Rgbcv = new RCDraw(paramsCamera.width,paramsCamera.height, qImageRGB, imgframe);
	
	qImageRGB2 = new QImage(paramsCamera.width,paramsCamera.height, QImage::Format_Indexed8);
	Rgbcv2 = new RCDraw(paramsCamera.width,paramsCamera.height, qImageRGB2, imgframe2);
	
	cvImage = cvCreateImage(cvSize(320,240), 8, 1);
	cvImage2 = cvCreateImage(cvSize(320,240), 8, 1);
	cvrgb = cvCreateImage(cvSize(320,240), 8, 1);
	cvrgb2 = cvCreateImage(cvSize(320,240), 8, 1);
	
	connect(buttonfear, SIGNAL(clicked(bool)),this, SLOT(movefear(bool)));
	connect(buttonsad, SIGNAL(clicked(bool)),this, SLOT(movesad(bool)));
	connect(buttonangry, SIGNAL(clicked(bool)),this, SLOT(moveangry(bool)));
	connect(buttonhappy, SIGNAL(clicked(bool)),this, SLOT(movehappy(bool)));
	connect(&timer, SIGNAL(timeout()), this, SLOT(compute()));
	timer.start(100); //is in ms
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}
void SpecificWorker::compute( )
{
	
	/////////////////IMU/////////////////////////////////////////////////////////
	datos = imu_proxy->getDataImu();
	
	pitch->display(datos.rot.Pitch);
	roll->display(datos.rot.Roll);
	yaw->display(datos.rot.Yaw);
	
	accx->display(datos.acc.XAcc);
	accy->display(datos.acc.YAcc);
	accz->display(datos.acc.ZAcc);
	
	magx->display(datos.mag.XMag);
	magy->display(datos.mag.YMag);
	magz->display(datos.mag.ZMag);
	
	gyrx->display(datos.gyro.XGyr);
	gyry->display(datos.gyro.YGyr);
	gyrz->display(datos.gyro.ZGyr);
	
	//qDebug()<<"phidgets"<<"pitch"<<datos.rot.Pitch<<"yaw"<<datos.rot.Yaw<<"roll"<<datos.rot.Roll;
	//qDebug()<<"phidgets2"<<"x"<<datos.acc.XAcc<<"y"<<datos.acc.YAcc<<"z"<<datos.acc.ZAcc;
	
	////////////////motores///////////////////////////////////////
			try{
		mutex->lock();
			jointmotor1_proxy->getAllMotorState(stateFaulhaber);
		mutex->unlock();
	}
	catch(Ice::Exception &ex)
	{
		std::cout << ex<<std::endl;
	}
	try{
		mutex->lock();
			jointmotor0_proxy->getAllMotorState(stateMuecas);
		mutex->unlock();
	}
	catch(Ice::Exception &ex)
	{
		std::cout << ex<<std::endl;
	}

	mutex_memory->lock();
		stateAll = stateFaulhaber;
		std::pair<std::string,RoboCompJointMotor::MotorState> pair;
		foreach (pair,stateMuecas)
			stateAll[pair.first] = pair.second;
	mutex_memory->unlock();
	
	///////////camara/////////////////////////////////////////////////
	try{
		
			camera_proxy->getYImage(5, imgCam, hState, bState);
  			memcpy(cvImage->imageData, &imgCam[0], imgSize);	
 			memcpy(cvImage2->imageData, &imgCam[imgSize], imgSize);	
  	}
  	catch(const Ice::Exception& e)
  	{
  		qDebug()<<"Error reading camera images";
  	}

 	cvFlip(cvImage, cvImage, 0);
 	cvFlip(cvImage2, cvImage2, 0);
 	memcpy (  qImageRGB->bits(), cvImage->imageData, paramsCamera.width*paramsCamera.height );
	memcpy (  qImageRGB2->bits(), cvImage2->imageData, paramsCamera.width*paramsCamera.height );

	
	if(checkBoxmobile->isChecked()){
	moveneck();
	movemobileelements();
	} else{
		
		//qDebug()<<"Only Joystick";
		
	}
	Rgbcv->update();
	Rgbcv2->update();

}
void SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
};




void SpecificWorker::resetHead(){
			try
	{
		saccadic4D( 0.f,0.f,0.f,0.f);
	}
	catch( RoboCompJointMotor::UnknownMotorException & ex)
	{
		std::cout << ex;
	}
	catch( RoboCompJointMotor::HardwareFailedException & ex)
	{
		std::cout << ex;
	}
}
void SpecificWorker::stopHead(){
}
void SpecificWorker::setPanLeft(float pan){
	
		RoboCompJointMotor::MotorGoalPosition g;
	g.name = leftPanMotorName;
	g.maxSpeed = 0;
	g.position = pan;
	QMutexLocker lock(mutex);
	try
	{
		jointmotor1_proxy->setPosition( g );
	}
	catch(RoboCompJointMotor::OutOfRangeException &ex)
	{
		throw ex;
	}
	catch(Ice::Exception &ex)
	{
		throw ex;
	}
}
void SpecificWorker::setPanRight(float pan){
	RoboCompJointMotor::MotorGoalPosition g;
	g.name =rightPanMotorName;
	g.maxSpeed = 0;
	g.position = pan;
	QMutexLocker lock(mutex);
	try
	{
		jointmotor1_proxy->setPosition( g );
	}
	catch(RoboCompJointMotor::OutOfRangeException &ex)
	{
		throw ex;
	}
	catch(Ice::Exception &ex)
	{
		throw ex;
	}
}
void SpecificWorker::setTilt(float tilt){
	RoboCompJointMotor::MotorGoalPosition g;
	g.name = tiltMotorName;
	g.maxSpeed = 0;
	g.position = tilt;
	QMutexLocker lock(mutex);
	try
	{
		jointmotor1_proxy->setPosition( g );
	}
	catch( RoboCompJointMotor::OutOfRangeException &ex )
	{
		throw ex;
	}
	catch(Ice::Exception &ex)
	{
		throw ex;
	}
}
void SpecificWorker::setNeck(float neck){
	RoboCompJointMotor::MotorGoalPosition g;
	g.name = neckMotorName;
	g.maxSpeed = 0;
	g.position = neck;
	QMutexLocker lock(mutex);
	try{
		jointmotor1_proxy->setPosition( g );
	}
	catch( RoboCompJointMotor::OutOfRangeException &ex )
	{
		throw ex;
	}
	catch(Ice::Exception &ex)
	{
		throw ex;
	}
}
void SpecificWorker::saccadic2DLeft(float leftPan, float tilt){
	RoboCompJointMotor::MotorGoalPositionList list;
	RoboCompJointMotor::MotorGoalPosition g;
	
	g.maxSpeed = 0;
	g.name = leftPanMotorName;
	g.position = leftPan;
	list.push_back( g );
	g.maxSpeed = 0;
	g.name = tiltMotorName;
	g.position = tilt;
	list.push_back( g );

	QMutexLocker lock(mutex);
	try
	{
		jointmotor1_proxy->setSyncPosition( list );
	}
	catch( RoboCompJointMotor::UnknownMotorException & ex)
	{
		throw ex;
	}
	catch( RoboCompJointMotor::HardwareFailedException & ex)
	{
		throw ex;
	}
}
void SpecificWorker::saccadic2DRight(float rightPan, float tilt){
	RoboCompJointMotor::MotorGoalPositionList list;
	RoboCompJointMotor::MotorGoalPosition g;
	
	g.maxSpeed = 0;
	g.name = rightPanMotorName;
	g.position = rightPan;
	list.push_back( g );
	g.maxSpeed = 0;
	g.name = tiltMotorName;
	g.position = tilt;
	list.push_back( g );
	QMutexLocker lock(mutex);
	try
	{
		jointmotor1_proxy->setSyncPosition( list );
	}
	catch( RoboCompJointMotor::UnknownMotorException & ex)
	{
		throw ex;
	}
	catch( RoboCompJointMotor::HardwareFailedException & ex)
	{
		throw ex;
	}
}
void SpecificWorker::saccadic3D(float leftPan, float rightPan, float tilt){
	RoboCompJointMotor::MotorGoalPositionList list;
	RoboCompJointMotor::MotorGoalPosition g;
	
	g.maxSpeed = 0;
	g.name = leftPanMotorName;
	g.position = leftPan;
	list.push_back( g );
	g.maxSpeed = 0;
	g.name = rightPanMotorName;
	g.position = rightPan;
	list.push_back( g );
	g.maxSpeed = 0;
	g.name = tiltMotorName;
	g.position = tilt;
	list.push_back( g );
	QMutexLocker lock(mutex);	
	try
	{
		jointmotor1_proxy->setSyncPosition( list );
	}
	catch( RoboCompJointMotor::UnknownMotorException & ex)
	{
		throw ex;
	}
	catch( RoboCompJointMotor::HardwareFailedException & ex)
	{
		throw ex;
	}
}
void SpecificWorker::saccadic4D(float leftPan, float rightPan, float tilt, float neck){
	RoboCompJointMotor::MotorGoalPositionList list;
	RoboCompJointMotor::MotorGoalPosition g;
	g.maxSpeed = 0;
	g.name = leftPanMotorName;
	g.position = leftPan;
	list.push_back( g );
	g.maxSpeed = 0;
	g.name = rightPanMotorName;
	g.position = rightPan;
	list.push_back( g );
	g.maxSpeed = 0;
	g.name = tiltMotorName;
	g.position = tilt;
	list.push_back( g );
	g.maxSpeed = 0;
	g.name = neckMotorName;
	g.position = neck;
	list.push_back( g );
	QMutexLocker lock(mutex);	
	try
	{
		jointmotor1_proxy->setSyncPosition( list );
	}
	catch( RoboCompJointMotor::UnknownMotorException & ex)
	{
		std::cout << ex;
		throw ex;
	}
	catch( RoboCompJointMotor::HardwareFailedException & ex)
	{
		std::cout << ex;
		throw ex;
	}
}
void SpecificWorker::setNMotorsPosition(const RoboCompJointMotor::MotorGoalPositionList& listGoals){
	RoboCompJointMotor::MotorGoalPositionList list;
	RoboCompJointMotor::MotorGoalPosition g;
	
	for(uint i=0; i<listGoals.size(); i++)
	{
		if(listGoals[i].name=="leftPan")
		{
			g.maxSpeed = 0;
			g.name = leftPanMotorName;
			g.position = listGoals[i].position;
			list.push_back(g);
		}
		else
		{
			if(listGoals[i].name=="rightPan")
			{
				g.maxSpeed = 0;
				g.name = rightPanMotorName;
				g.position = listGoals[i].position;
				list.push_back(g);
			}
			else
			{
				if(listGoals[i].name=="tilt")
				{
					g.maxSpeed = 0;
					g.name = tiltMotorName;
					g.position = listGoals[i].position;
					list.push_back(g);
				}
				else
				{
					if(listGoals[i].name=="neck")
					{
						g.maxSpeed = 0;
						g.name = neckMotorName;
						g.position = listGoals[i].position;
						list.push_back(g);
					}
				}
			}
		}
	}
	if(list.size()>0)
	{
		QMutexLocker lock(mutex);
		try
		{
			jointmotor1_proxy->setSyncPosition( list );
		}
		catch( RoboCompJointMotor::UnknownMotorException & ex)
		{
			throw ex;
		}
		catch( RoboCompJointMotor::HardwareFailedException & ex)
		{
			throw ex;
		}
	}
}
RoboCompCommonHead::THeadParams SpecificWorker::getHeadParams(){
	return headParams;
	//return 0;
}
void SpecificWorker::getHeadState(RoboCompCommonHead::THeadState& hState){
	mutex_memory->lock();
/*	qDebug()<<"get head state"<<&stateAll;
	qDebug()<<"size"<<stateAll.size();*/
	
	
	hState.motorsState[leftPanMotorName] = stateAll[leftPanMotorName];
	hState.motorsState[rightPanMotorName] = stateAll[rightPanMotorName];
	hState.motorsState[neckMotorName] = stateAll[neckMotorName];
	hState.motorsState[tiltMotorName] = stateAll[tiltMotorName];
	
	mutex_memory->unlock();
	
	hState.isMoving = hState.motorsState[leftPanMotorName].isMoving or hState.motorsState[rightPanMotorName].isMoving or
		hState.motorsState[neckMotorName].isMoving or hState.motorsState[tiltMotorName].isMoving;
	headmoving = hState.isMoving;
}
bool SpecificWorker::isMovingHead(){
	return headmoving;
	//return 0;
}


RoboCompJointMotor::MotorParamsList SpecificWorker::getAllMotorParams()
{
	return mplAll;
}
void  SpecificWorker::getAllMotorState(RoboCompJointMotor::MotorStateMap& mstateMap)
{
	mutex_memory->lock();
		mstateMap = stateAll;
	mutex_memory->unlock();
}
void  SpecificWorker::setPosition(const RoboCompJointMotor::MotorGoalPosition& goal)
{
	QMutexLocker lock(mutex);
	try
	{
		jointmotor1_proxy->setPosition( goal );
	
		
	}
	catch( RoboCompJointMotor::UnknownMotorException & ex)
	{
		throw ex;
	}
	catch( RoboCompJointMotor::HardwareFailedException & ex)
	{
		throw ex;
	}
}


	void SpecificWorker::moveangry(bool)
	{

	move(0.01, mouthMotorName);
	
	move(0.01, eyeBLeftYAWMotorName );

    move(0.01, eyeBLeftUPMotorName );
	
	move(0.01, eyeBRightUPMotorName );

    move(-0.01, eyeBRightYAWMotorName );
	

	}
	
	
	void SpecificWorker::movehappy(bool)
	{		
		
	move(-0.02, mouthMotorName);
	
	move(0.02, eyeBLeftYAWMotorName );

    move(-0.02, eyeBLeftUPMotorName );
	
	move(-0.02, eyeBRightUPMotorName );

    move(-0.02, eyeBRightYAWMotorName );
		

	}

	
	void SpecificWorker::movefear(bool)
	{
	
	move(-0.02, mouthMotorName);
	
	move(-0.02, eyeBLeftYAWMotorName );

    move(-0.02, eyeBLeftUPMotorName );
	
	move(-0.02, eyeBRightUPMotorName );

    move(0.02, eyeBRightYAWMotorName );

	}
	
	
	void SpecificWorker::movesad(bool)
	{
	
	move(0.02, mouthMotorName);
	
	move(-0.02, eyeBLeftYAWMotorName );

    move(0.02, eyeBLeftUPMotorName );
	
	move(0.02, eyeBRightUPMotorName );

    move(0.02, eyeBRightYAWMotorName );
	
	}

	

void SpecificWorker::move(float pos,  const string& Name )
{


RoboCompJointMotor::MotorGoalPosition g;
 	g.name = Name;
 	g.maxSpeed = 0;
 	g.position = pos;
 		QMutexLocker lock(mutex);
 	try{
 		jointmotor0_proxy->setPosition( g );
 	}
 	catch( RoboCompJointMotor::OutOfRangeException &ex )
 	{
 		throw ex;
 	}
 	catch(Ice::Exception &ex)
 	{
 		throw ex;
 	}
	
}



void SpecificWorker::moveneck()
{
		float rotations_eyeA = righteyepan->value();
		float rotations_eyeB = lefteyepan->value();
		
		float eyestilt = eyetilt->value();
		float ROTATE = neckyaw->value();
		
		float POS_Zz = neckpitch->value();
		float POS_Xx = -neckpitch->value() + neckroll->value();
		float POS_Yy = -neckpitch->value() - neckroll->value();
		
		int pasos_rotate = ROTATE * 17960.f / 6.28;
		int pasos_zz = POS_Zz * 17960.f / 6.28; 
		int pasos_xx = POS_Xx * 17960.f / 6.28;
		int pasos_yy = POS_Yy * 17960.f / 6.28;
		
		qDebug()<<"pasos"<<pasos_zz<<pasos_xx<<pasos_yy<<pasos_rotate;
		
			
			// limites de los recorridos de los motores
			if ( pasos_xx > MIN_X && pasos_xx < MAX_X && pasos_yy > MIN_Y && pasos_yy < MAX_Y && pasos_zz > MIN_Z && pasos_zz < MAX_Z && pasos_rotate > MIN_TILT &&  pasos_rotate < MAX_TILT)
			{
				qDebug()<<"stting positios"<<"front"<<POS_Zz<<"right"<<POS_Xx<<"left"<<POS_Yy;
				RoboCompJointMotor::MotorGoalPosition p_goal;
				RoboCompJointMotor::MotorGoalPositionList list;
				
				p_goal.name = "neckRight";
				p_goal.position = POS_Xx;
				list.push_back(p_goal);
				
				p_goal.name = "neckLeft";
				p_goal.position = POS_Yy;
				list.push_back(p_goal);
				
				p_goal.name = "neckFront";
				p_goal.position = POS_Zz;
				list.push_back(p_goal);

				
				p_goal.name = "neck";
				p_goal.position = ROTATE;
				list.push_back(p_goal);
				
				
				p_goal.name = "rightPan";
				p_goal.position = rotations_eyeA;
				list.push_back(p_goal);
				
							
				p_goal.name = "leftPan";
				p_goal.position = rotations_eyeB;
				list.push_back(p_goal);
				
				
				p_goal.name = "tilt";
				p_goal.position = eyestilt;
				list.push_back(p_goal);
			
				mutex->lock();
				try
				{
					jointmotor1_proxy->setSyncPosition( list );
				}
				catch( Ice::Exception & ex)
				{
					std::cout<< ex.what()<< std::endl;
				}
				mutex->unlock();
			}
		
}

void SpecificWorker::movemobileelements()
{
	move(mouth->value()/100, mouthMotorName);
	
	move(-lefteyebrownroll->value()/100, eyeBLeftYAWMotorName );
	
	move(righteyebrownroll->value()/100, eyeBRightYAWMotorName );

    move(lefteyebrownpitch->value()/100, eyeBLeftUPMotorName );
	
	move(righteyebrownpitch->value()/100, eyeBRightUPMotorName );

}


#define max_pasos = 

///JoystickAdapter
void SpecificWorker::sendData(const TData& data)
{
	qDebug()<<"jdata0"<<data.axes[0].value;
	qDebug()<<"jdata1"<<data.axes[1].value;
	qDebug()<<"jdata2"<<data.axes[2].value;
	if (data.buttons[0].clicked)
	{
		qDebug()<<"button 0 clicked";
		
		
		float joy_y = normalize(data.axes[1].value, -1., 1., -0.6,0.5);
		float joy_x = normalize(data.axes[0].value, -1., 1., -0.5,0.5);
		float rotate = normalize(data.axes[2].value, -1., 1., -0.5,0.5);
		
		qDebug()<<"normalize position 0"<<"pitch"<<joy_x<<"yaw"<<joy_y;
		
		float POS_Z = joy_y;
		float POS_X = -joy_y + joy_x;
		float POS_Y = -joy_y - joy_x;
		
		int step_rotate = rotate * 17960.f / 6.28;
		int pasos_z = POS_Z * 17960.f / 6.28; 
		int pasos_x = POS_X * 17960.f / 6.28;
		int pasos_y = POS_Y * 17960.f / 6.28;
		
		qDebug()<<"pasos"<<pasos_z<<pasos_x<<pasos_y<<step_rotate;
		
		if (data.axes[1].value!=joy_x_antiguo || data.axes[0].value != joy_y_antiguo || data.axes[2].value != joy_z_antiguo)
		{
			
			// limites de los recorridos de los motores
			if ( pasos_x > MIN_X && pasos_x < MAX_X && pasos_y > MIN_Y && pasos_y < MAX_Y && pasos_z > MIN_Z && pasos_z < MAX_Z && step_rotate > MIN_TILT &&  step_rotate < MAX_TILT)
			{
				qDebug()<<"stting positios"<<"front"<<POS_Z<<"right"<<POS_X<<"left"<<POS_Y;
				RoboCompJointMotor::MotorGoalPosition p_goal;
				RoboCompJointMotor::MotorGoalPositionList list;
				
				p_goal.name = "neck";
				p_goal.position = rotate;
				list.push_back(p_goal);
				
				p_goal.name = "neckRight";
				p_goal.position = POS_X;
				list.push_back(p_goal);
				
				p_goal.name = "neckLeft";
				p_goal.position = POS_Y;
				list.push_back(p_goal);
				
				p_goal.name = "neckFront";
				p_goal.position = POS_Z;
				list.push_back(p_goal);
 
				joy_x_antiguo = data.axes[1].value;
				joy_y_antiguo = data.axes[0].value;
				joy_z_antiguo = data.axes[2].value;
				
			
				mutex->lock();
				try
				{
					jointmotor1_proxy->setSyncPosition( list );
				}
				catch( Ice::Exception & ex)
				{
					std::cout<< ex.what()<< std::endl;
				}
				mutex->unlock();
			}
		}
        
	}
	else if (data.buttons[1].clicked)
	{
		// eyebrown and mouth
		qDebug()<<"button 1 clicked";
		
		float axes_y = normalize(data.axes[1].value, -1., 1., -0.6,0.5);
		float axes_x = normalize(data.axes[0].value, -1., 1., -0.5,0.5);
		float axes_mouth = normalize(data.axes[2].value, -1., 1., -0.5,0.5);
		
		qDebug()<<"normalize position 1"<<"pitch"<<axes_x<<"yaw"<<axes_y<<"roll"<<axes_mouth;
		
		move(axes_mouth/100, mouthMotorName);
	
		move(-axes_x/100, eyeBLeftYAWMotorName );
	
		move(axes_x/100, eyeBRightYAWMotorName );

		move(axes_y/100, eyeBLeftUPMotorName );
	
		move(axes_y/100, eyeBRightUPMotorName );
				
	}
	else if (data.buttons[2].clicked)
	{
				qDebug()<<"button 2 clicked";
		float rot_tilt = normalize(data.axes[1].value, -1., 1., -0.5,0.5) /3;
		float rotations_eyeA = normalize(data.axes[0].value, -1., 1., -0.5,0.5) /2;
		
				RoboCompJointMotor::MotorGoalPosition p_goal;
				RoboCompJointMotor::MotorGoalPositionList list;
		
				p_goal.name = "rightPan";
				p_goal.position = rotations_eyeA;
				list.push_back(p_goal);
				
							
				p_goal.name = "leftPan";
				p_goal.position = rotations_eyeA;
				list.push_back(p_goal);
				
				
				p_goal.name = "tilt";
				p_goal.position = rot_tilt;
				list.push_back(p_goal);
			
				mutex->lock();
				try
				{
					jointmotor1_proxy->setSyncPosition( list );
				}
				catch( Ice::Exception & ex)
				{
					std::cout<< ex.what()<< std::endl;
				}
				mutex->unlock();
		
	}
		
}

float SpecificWorker::normalize(float X, float A, float B, float C, float D)
{
	return ((D-C)*(X-A)/(B-A))+C;
}

