/*
 *    Copyright (C) 2010 by RoboLab - University of Extremadura
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
#include "dunkermotoren.h"
#include "dunkermotoren_api.h"
#include <qlog/qlog.h>


Dunkermotoren::Dunkermotoren(RoboCompJointMotor::BusParams *busParams, RoboCompJointMotor::MotorParamsList *params, QHash<int, DunkerParams> *_dunkerParams, QMutex *m) 
{
	this->busParams = busParams;
	this->params = params;
	this->dunkerParams = _dunkerParams;
	memory_mutex = m;
	hardware_mutex = new QMutex();
}

Dunkermotoren::~Dunkermotoren()
{
	
}

void Dunkermotoren::initialize() throw (QString)
{
	int ret = -1;
	QString device;
	//LEIDO DESDE EL FICHERO DE CONFIGURACION POR MONITOR
	device = QString::fromStdString( busParams->device);
	
	// Open and initialize the device
	qDebug() << "Dunkermotoren::initialize(): opening"<<device.toAscii().data();
	devHandler = VSCAN_Open(device.toAscii().data(),  VSCAN_MODE_NORMAL);
	qDebug() << "Dunkermotoren::initialize(): opened";
	if(devHandler<0)
	{
		qDebug() << "Dunkermotoren::initialize(): Failed to open Can device "<< device <<". Handler "<< devHandler;
		return;
	}
	qDebug()<<"Dunkermotoren::initialize(): Can device"<<device<<"oppened with handler"<<devHandler;
	qDebug()<<"Setting 125K speed";

	// SETTING BAUDRATE
printf("_____ %s: %d\n", __FILE__, __LINE__);
	Dunker_setBaudRate(devHandler, busParams->baudRate);
	
printf("_____ %s: %d\n", __FILE__, __LINE__);
// 	Dunker_JoseMateos(devHandler);

printf("_____ %s: %d\n", __FILE__, __LINE__);

	///Create servos instances in a QMap indexed by name
	for (int i = 0; i < busParams->numMotors; i++)
	{
		QString name = QString::fromStdString(params->operator[](i).name);
		motors[name] = new Servo( params->operator[](i) );
		//FIXME steps_range , max_degrees, steps_speed_range, max_speed_rads  ¿¿Deberían venir en los parámetros?
		//TODO: cambiar por param de config
		//Dunker direccion 159744
		motors[name]->setMotorRanges(params->operator[](i).stepsRange, 360, 500, 2.f);
	}

printf("_____ %s: %d\n", __FILE__, __LINE__);

	std::cout <<"Dunkermotoren::initialize(): - Motor Map created with " << busParams->numMotors << " motors: " << std::endl;
	foreach( Servo * s, motors)
	{
		std::cout << "	" + s->params.name << std::endl; 
	}
	
	
printf("_____ %s: %d\n", __FILE__, __LINE__);
	

	// CONSTRUCT MOTORID ARRAY
	///Initialize motor params
	
	int numMotors = motors.size();
	uint8_t NodeIds[numMotors];
	int k=0;
	foreach( Servo *s, motors)
	{
		NodeIds[k] = s->params.busId;
		k++;
	}
	//~ int ret;
	//~ if ( (ret=Dunker_syncMoveToRelativePosition(this->devHandler, numMotors, NodeIds, poss)) != 0 )
	  //~ throw QString("Dynamixel::setSyncPosition() - Error writing to port");
	/*
	 * Dunker_setDinamicLimitations(Handle, NodeId);
	 * 		Current Dynamic limitation
				Curr Limit Pos 0x3221,0x00,60000
				Curr Limit Neg 0x3223,0x00,60000
			CurrDynLimitOff 0x3224, 0x00, 0x00000000
			CurrDynLimitOn 0x3224, 0x00, 0x00000001
	//CurrDynLimitPeak(mA) - Peak current
	msgs[4] = Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3224, 0x01, 60000);
	//CurrDynLimitCont(mA) - Continous current
	msgs[5] = Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3224, 0x02, 10000);
	//CurrDynLimitTime(ms) - Time for Peak current
	msgs[6] = Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3224, 0x03, 2000);
	int status = Dunker_multiWriteWaitReadMessage(Handle, msgs, 7);
	 * Dunker_clearActualPosition(Handle, NodeId);
	 * Dunker_configureErrorsAndFeedback(Handle, NodeId);
	 * Dunker_clearError(Handle, NodeId);
	 */
	
printf("_____ %s: %d\n", __FILE__, __LINE__);


	//TODO: desglosar llamadas del reserModule
	qDebug() <<"Dunkermotoren::initialize(): Reseting motors OK.";
	if( (ret  = Dunker_syncResetModule(this->devHandler, motors.size(), NodeIds )) != 0 )
		qDebug() <<"Dunkermotoren::initialize(): Error: Dunker_syncResetModule. result="<<ret;
	else
		qDebug() <<"Dunkermotoren::initialize(): Dunker_resetModule OK.";
	
	int maxPos[motors.size()];
	int minPos[motors.size()];
	int maxVelPos[motors.size()];
	int maxVelNeg[motors.size()];
	int posCurLimit[motors.size()];
	int negCurLimit[motors.size()];
	int dynCurPeak[motors.size()];
	int dynCurContin[motors.size()];
	int dynCurTime[motors.size()];
	int encoderRes[motors.size()];
	int velAcc[motors.size()];
	int timeAcc[motors.size()];
	int velDec[motors.size()];
	int timeDec[motors.size()];
	int setPID[motors.size()];
	int velKps[motors.size()];
	int velKis[motors.size()];
	int velKds[motors.size()];
	
printf("_____ %s: %d\n", __FILE__, __LINE__);

	k=0;
	foreach( Servo *s, motors )
	{
		
		RoboCompJointMotor::MotorParams &params = s->params;
		DunkerParams dparams = dunkerParams->operator[](params.busId);

		std::cout << "Dunkermotoren::initialize(): - Configuration data of motor " << params.name << std::endl;
		
		maxPos[k] = s->rads2Steps(params.maxPos);
		std::cout <<"MaxPos in "<<params.maxPos<<std::endl;
		std::cout <<"MaxPos to set "<<maxPos[k]<<std::endl;
		minPos[k] = s->rads2Steps(params.minPos);
		std::cout <<"MinPos in "<<params.minPos<<std::endl;
		std::cout <<"MinPos  to set "<<minPos[k]<<std::endl;
		maxVelPos[k] = s->rads2Steps(params.maxVelocity);
		std::cout <<"MaxVelPos in "<<params.maxVelocity<<std::endl;
		std::cout <<"MaxVelPos  to set "<<maxVelPos[k]<<std::endl;
		maxVelNeg[k] = -s->rads2Steps(params.maxVelocity);
		std::cout <<"MaxVelNeg in "<<params.maxVelocity<<std::endl;
		std::cout <<"MaxVelNeg  to set "<<maxVelNeg[k]<<std::endl;
		posCurLimit[k] = dparams.posCurLim;
		negCurLimit[k] = dparams.negCurLim;
		dynCurPeak[k] = dparams.curPeak;
		dynCurContin[k] = dparams.curContin;
		dynCurTime[k] = dparams.curTime;
		encoderRes[k] = dparams.encoderRes;
		velAcc[k] = dparams.accV;
		timeAcc[k] = dparams.accT;
		velDec[k] = dparams.decV;
		timeDec[k] = dparams.decT;
		setPID[k] = dparams.setPID;
		std::cout <<"setPID "<<setPID[k]<<std::endl;
		velKps[k] = dparams.velKp;
		std::cout <<"velKps "<<velKps[k]<<std::endl;
		velKis[k] = dparams.velKi;
		std::cout <<"velKis "<<velKis[k]<<std::endl;
		velKds[k] = dparams.velKd;
		std::cout <<"velKds "<<velKds[k]<<std::endl;
		
printf("_____ %s: %d\n", __FILE__, __LINE__);
		
		if ((ret=Dunker_setOperationMode(this->devHandler, params.busId, MODE_POS)) != 0)
		{
			throw QString("Dunkermotoren::setPosition() - Error changing mode ("+QString::number(MODE_POS)+"): "+QString::number(ret));
		}
		else
		{
			s->data.mode = MODE_POS;
		}
		k++;
	}
	
printf("_____ %s: %d\n", __FILE__, __LINE__);

	for (int patata=0; patata<4; patata++)
	{
		qDebug()<<"maxVelPos[k]"<<QString::number(maxVelPos[patata]);
		qDebug()<<"maxVelNeg[k]"<<QString::number(maxVelNeg[patata]);
		qDebug()<<"posCurLimit[k]"<<QString::number(posCurLimit[patata]);
		qDebug()<<"negCurLimit[k]"<<QString::number(negCurLimit[patata]);
		qDebug()<<"dynCurPeak[k]"<<QString::number(dynCurPeak[patata]);
		qDebug()<<"dynCurContin[k]"<<QString::number(dynCurContin[patata]);
		qDebug()<<"dynCurTime[k]"<<QString::number(dynCurTime[patata]);
		qDebug()<<"encoderRes[k]"<<QString::number(encoderRes[patata]);
		qDebug()<<"-------------";
	}
	
printf("_____ %s: %d\n", __FILE__, __LINE__);

	//TODO: CHAPUZA, HABRIA QUE COMPROBARLO PARA CADA MOTOR
        if((ret = Dunker_syncSetVelKps(this->devHandler, motors.size(), NodeIds, velKps)) == 0){
		qDebug()<<"************VEL KPS**********";
		for (uint i= 0;i<motors.size();i++)
			qDebug()<<"motor "<<i<<"value"<<velKps[i];
	}


	if(setPID[0]!=0)
	{
		if((ret = Dunker_syncSetVelKps(this->devHandler, motors.size(), NodeIds, velKps)) == 0)
		{
			qDebug() << "Dunkermotoren::initialize(): VelKps set";
                     
		}
		else
		{
			qDebug() << "Dunkermotoren::initialize(): ERROR Setting VelKps:"<<ret;
		}
		
		if((ret = Dunker_syncSetVelKis(this->devHandler, motors.size(), NodeIds, velKis)) == 0)
		{
			qDebug() << "Dunkermotoren::initialize(): VelKis set";
		}
		else
		{
			qDebug() << "Dunkermotoren::initialize(): ERROR Setting VelKis:"<<ret;
		}
		
		if((ret = Dunker_syncSetVelKds(this->devHandler, motors.size(), NodeIds, velKds)) == 0)
		{
			qDebug() << "Dunkermotoren::initialize(): VelKds set";
		}
		else
		{
			qDebug() << "Dunkermotoren::initialize(): ERROR Setting VelKds:"<<ret;
		}
	}
	
	if((ret = Dunker_syncSetVel(this->devHandler, motors.size(), NodeIds, maxVelPos)) == 0)
	{
		qDebug() << "Dunkermotoren::initialize(): Max velocity set";
	}
	else
	{
		qDebug() << "Dunkermotoren::initialize(): ERROR Setting Max velocity:"<<ret;
	}
	
	if((ret = Dunker_syncSetMaxPos(this->devHandler, motors.size(), NodeIds, maxPos)) == 0)
	{
		qDebug() << "Dunkermotoren::initialize(): Max position set";
	}
	else
	{
		qDebug() << "Dunkermotoren::initialize(): ERROR Setting Max position:"<<ret;
	}
	if((ret = Dunker_syncSetMinPos(this->devHandler, motors.size(), NodeIds, minPos)) == 0)
	{
		qDebug() << "Dunkermotoren::initialize(): Min positions set.";
	}
	else
	{
		qDebug() << "Dunkermotoren::initialize(): ERROR setting Min Positions: "<<ret;
	}
	if((ret = Dunker_syncSetMaxVelPositive(this->devHandler, motors.size(), NodeIds, maxVelPos)) == 0)
	{
		qDebug() << "Dunkermotoren::initialize(): Max Positive Velocities set.";
	}
	else
	{
		qDebug() << "Dunkermotoren::initialize(): ERROR setting Max Postivive Velocities: "<<ret;
	}
	if((ret = Dunker_syncSetMaxVelNegative(this->devHandler, motors.size(), NodeIds, maxVelNeg)) == 0)
	{
		qDebug() << "Dunkermotoren::initialize(): Max Negative Velocities set.";
	}
	else
	{
		qDebug() << "Dunkermotoren::initialize(): ERROR setting Max Negative Velocities:"<<ret;
	}
	
	
	if((ret = Dunker_syncPositiveCurrentLimit(this->devHandler, motors.size(), NodeIds, posCurLimit)) == 0)
	{
		qDebug() << "Dunkermotoren::initialize(): Positive Current Limit set.";
	}
	else
	{
		qDebug() << "Dunkermotoren::initialize(): ERROR setting Positive Current Limit:"<<ret;
	}
	
	if((ret = Dunker_syncNegativeCurrentLimit(this->devHandler, motors.size(), NodeIds, negCurLimit)) == 0)
	{
		qDebug() << "Dunkermotoren::initialize(): Negative Current Limit set.";
	}
	else
	{
		qDebug() << "Dunkermotoren::initialize(): ERROR setting Negative Current Limit:"<<ret;
	}
	
	if((ret = Dunker_syncDynCurrentLimitPeak(this->devHandler, motors.size(), NodeIds, dynCurPeak)) == 0)
	{
		qDebug() << "Dunkermotoren::initialize(): Dynamic Current Peak set.";
	}
	else
	{
		qDebug() << "Dunkermotoren::initialize(): ERROR setting Dynamic Current Peak:"<<ret;
	}
	
	if((ret = Dunker_syncDynCurrentLimitContinous(this->devHandler, motors.size(), NodeIds, dynCurContin)) == 0)
	{
		qDebug() << "Dunkermotoren::initialize(): Dynamic Continues Current set.";
	}
	else
	{
		qDebug() << "Dunkermotoren::initialize(): ERROR setting Dynamic Continues Current "<<ret;
	}
	if((ret = Dunker_syncDynCurrentLimitTime(this->devHandler, motors.size(), NodeIds, dynCurTime)) == 0)
	{
		qDebug() << "Dunkermotoren::initialize(): Dynamic Current Time set.";
	}
	else
	{
		qDebug() << "Dunkermotoren::initialize(): ERROR setting Dynamic Current Time"<<ret;
	}
	////ACC Y DEC
	qDebug() << "#######################################################";
	qDebug() << "#######################################################";
	qDebug() << "#######################################################";
	qDebug() << "#######################################################";
	qDebug() << "#######################################################";
	if((ret = Dunker_syncSetAcceleration(this->devHandler, motors.size(), NodeIds, velAcc, timeAcc)) == 0)
	{
		qDebug() << "Dunkermotoren::initialize(): Sync Acceleration set.";
	}
	else
	{
		qDebug() << "Dunkermotoren::initialize(): ERROR setting sync Acceleration"<<ret;
	}
	if((ret = Dunker_syncSetDecceleration(this->devHandler, motors.size(), NodeIds, velDec, timeDec)) == 0)
	{
		qDebug() << "Dunkermotoren::initialize(): Sync Deceleration set.";
	}
	else
	{
		qDebug() << "Dunkermotoren::initialize(): ERROR setting sync Deceleration"<<ret;
	}
	

	
//printf("_____ %s: %d\n", __FILE__, __LINE__);
//printf("_____ %s: %d\n", __FILE__, __LINE__);
foreach( Servo *s, motors)
{
	uint8_t id = s->params.busId;
	if ((ret = Dunker_setAcceleration(this->devHandler, id, 70, 100)) == 0)
	{
		qDebug() << "Dunkermotoren::initialize(): Acceleration set.";
	}
	else
	{
		qDebug() << "Dunkermotoren::initialize(): ERROR setting Acceleration"<<ret;
	}
	if ((ret = Dunker_setDecceleration(this->devHandler, id, 70, 100)) == 0)
	{
		qDebug() << "Dunkermotoren::initialize(): Deceleration set.";
	}
	else
	{
		qDebug() << "Dunkermotoren::initialize(): ERROR setting Deceleration"<<ret;
	}
}
//printf("_____ %s: %d\n", __FILE__, __LINE__);
//printf("_____ %s: %d\n", __FILE__, __LINE__);
	

	
	//~ if((ret = Dunker_syncSetEncoderRes(this->devHandler, motors.size(), NodeIds, encoderRes)) == 0)
	//~ {
		//~ qDebug() << "Dunkermotoren::initialize(): Encoder resolution set.";
	//~ }
	//~ else
	//~ {
		//~ qDebug() << "Dunkermotoren::initialize(): ERROR setting Encoder resolution"<<ret;
	//~ }
	
	
printf("_____ %s: %d\n", __FILE__, __LINE__);

	foreach(Servo *s, motors)
	{
		int pos = -1;
		///Read current position		
		RoboCompJointMotor::MotorParams &params = s->params;
		Servo::TMotorData &data = s->data;
		qDebug() << "Motor" << QString::fromStdString(params.name);
		if((ret = Dunker_getPos(this->devHandler, params.busId, &pos)) == 0)
		{
			qDebug() << "	Current position (read): " << pos;
			data.currentPos = pos;
			qDebug() << "	Current position: " << data.currentPos;
		}
		else
		{
			qDebug() << "	ERROR ("<<ret<<")reading current position: "<<pos<<endl;

		}
		pos=-1;
		if((ret = Dunker_getMaxPos(this->devHandler, params.busId, &pos)) == 0)
		{
			qDebug() << "	Max position read (steps): " << pos;
		}
		else
		{
			qDebug() << "	ERROR ("<< ret <<") reading Max position.";
		}
		pos=-1;
		if(Dunker_getMinPos(this->devHandler, params.busId, &pos ) == 0)
		{
			qDebug() << "	Min position read (steps): " << pos;
		}
		else
		{
			qDebug() << "	ERROR reading Min position";
		}
		
		pos=-1;
		int IncVelPositive =-1;
		int IncVelNegative = -1;
		if(Dunker_getMaxVelPositive(this->devHandler, params.busId, &IncVelPositive ) == 0)
		{
			if(Dunker_getMaxVelNegative(this->devHandler, params.busId, &IncVelNegative ) == 0)
			{
				qDebug() << "	Max Velocity read (steps): "<< IncVelPositive <<" <<==|==>> " << IncVelNegative;
			}
			else
			{
				qDebug() << "	ERROR reading Max. Velocity Negative";
			}
		}
		else
		{
			qDebug() << "	ERROR reading Max. Velocity Positive ";
		}
	}
	
	
	
printf("_____ %s: %d\n", __FILE__, __LINE__);
	
	
	if((ret = Dunker_syncDisablePower(this->devHandler, motors.size(), NodeIds)) == 0)
	{
		qDebug() << "Dunkermotoren::initialize(): Power disabled";
	}
	else
	{
		qDebug() << "Dunkermotoren::initialize(): ERROR disabling Power:"<<ret;
	}
	if((ret = Dunker_syncOpenBreak(this->devHandler, motors.size(), NodeIds)) == 0)
	{
		qDebug() << "Dunkermotoren::initialize(): Break open";
	}
	else
	{
		qDebug() << "Dunkermotoren::initialize(): ERROR opening Break:"<<ret;
	}

	if((ret = Dunker_syncClearError(this->devHandler, motors.size(), NodeIds)) == 0)
	{
		qDebug() << "Dunkermotoren::initialize(): error cleared.";
	}
	else
	{
		qDebug() << "Dunkermotoren::initialize(): ERROR clearing error:"<<ret;
	}
	
	//~ if((ret = Dunker_syncSetDinamicLimitations(this->devHandler, motors.size(), NodeIds)) == 0)
	//~ {
		//~ qDebug() << "Dunkermotoren::initialize(): dynamic limitations set.";
	//~ }
	//~ else
	//~ {
		//~ qDebug() << "Dunkermotoren::initialize(): ERROR setting dynamic limitations:"<<ret;
	//~ }
	
	if((ret = Dunker_syncConfigureErrorsAndFeedback(this->devHandler, motors.size(), NodeIds)) == 0)
	{
		qDebug() << "Dunkermotoren::initialize(): Error & feedback configured.";
	}
	else
	{
		qDebug() << "Dunkermotoren::initialize(): ERROR configuring error & feedback:"<<ret;
	}
	
printf("_____ %s: %d\n", __FILE__, __LINE__);

	//TODO: cargar del fichero de config
	if((ret = Dunker_syncSetMaxPosFollowingError(this->devHandler, motors.size(), NodeIds,10000)) == 0)
	{
		qDebug() << "Dunkermotoren::initialize(): Maximum position folowing error set.";
	}
	else
	{
		qDebug() << "Dunkermotoren::initialize(): ERROR setting Maximum position folowing error configured:"<<ret;
	}
	
	if((ret = Dunker_syncSetRampType(this->devHandler, motors.size(), NodeIds,1)) == 0)
	{
		qDebug() << "Dunkermotoren::initialize(): Ramp type set to 1.";
	}
	else
	{
		qDebug() << "Dunkermotoren::initialize(): ERROR setting ramp type 1"<<ret;
	}
	
	foreach(Servo *s, motors)
	{
		
		RoboCompJointMotor::MotorParams &params = s->params;
		DunkerParams &dparams = dunkerParams->operator[](params.busId);
		//TODO: cargar del fichero de config
		//~ if((ret = Dunker_setAcceleration(this->devHandler, params.busId, 1000, 300)) == 0)
		//~ {
			//~ qDebug() <<"Acceleration set";
		//~ }
		//~ else
		//~ {
			//~ qDebug() << "ERROR ("<<ret<<") setting acceleration";
//~ 
		//~ }
		//~ //TODO: cargar del fichero de config
		//~ if((ret = Dunker_setDecceleration(this->devHandler, params.busId, 1000, 300)) == 0)
		//~ {
			//~ qDebug() <<"Decceleration set";
		//~ }
		//~ else
		//~ {
			//~ qDebug() << "ERROR ("<<ret<<") setting decceleration";
		//~ }
		//TODO: cargar del fichero de config
		
		if((ret = Dunker_setGearSaftRevolution(this->devHandler, params.busId, dparams.gearRev)) == 0)
		{
			qDebug() <<"Gear Revolution set";
		}
		else
		{	
			qDebug() << "ERROR ("<<ret<<") setting revolution";

		}
		//TODO: cargar del fichero de config
		if((ret = Dunker_setMotorSaftRevolution(this->devHandler, params.busId, dparams.motorRev)) == 0)
		{
			qDebug() <<"Motor Revolution set";
		}
		else
		{
			qDebug() << "ERROR ("<<ret<<") setting revolution";
		}
	}
	
printf("_____ %s: %d\n", __FILE__, __LINE__);
	
	//~ UINT32 accV[4]={900, 920, 930, 940};
	//~ UINT32 accT[4]={110, 120, 130, 140};
	//~ if((ret = Dunker_syncSetAcceleration(this->devHandler, motors.size(), NodeIds, accV, accT)) == 0)
	//~ {
		//~ qDebug() << "Dunkermotoren::initialize(): Acceleration set.";
	//~ }
	//~ else
	//~ {
		//~ qDebug() << "Dunkermotoren::initialize(): ERROR setting acceleration set:"<<ret;
	//~ }
	//~ UINT32 decV[4]={1000, 1000, 1000, 1000};
	//~ UINT32 decT[4]={100, 100, 100, 100};
	//~ if((ret = Dunker_syncSetDecceleration(this->devHandler, motors.size(), NodeIds, decV, decT)) == 0)
	//~ {
		//~ qDebug() << "Dunkermotoren::initialize(): Decceleration set.";
	//~ }
	//~ else
	//~ {
		//~ qDebug() << "Dunkermotoren::initialize(): ERROR setting acceleration:"<<ret;
	//~ }
	
	//~ Dunker_setCurrentLimit(this->devHandler, params.busId,60000);
	if((ret = Dunker_syncEnablePower(this->devHandler, motors.size(), NodeIds)) == 0)
	{
		qDebug() << "Dunkermotoren::initialize(): Power enabled.";
	}
	else
	{
		qDebug() << "Dunkermotoren::initialize(): ERROR enabling Power:"<<ret;
	}

printf("_____ %s: %d\n", __FILE__, __LINE__);
	
}

bool Dunkermotoren::reset(uchar motor)
{
	qDebug()<<"reseting motor "<<motor;
	DunkerParams &dparams = dunkerParams->operator[](motor);
	Dunker_disablePower(this->devHandler, motor);
	Dunker_openBreak(this->devHandler, motor);
	Dunker_clearError(this->devHandler, motor);
	//Dunker_setDinamicLimitations(this->devHandler, motor);
	Dunker_setDynCurrentLimitOff(this->devHandler, motor);
	Dunker_setPositiveCurrentLimit(this->devHandler, motor, dparams.posCurLim);
	Dunker_setNegativeCurrentLimit(this->devHandler, motor, dparams.negCurLim);
	Dunker_setDynCurrentLimitOn(this->devHandler, motor);
	Dunker_setDynCurrentLimitPeak(this->devHandler, motor, dparams.curPeak);
	Dunker_setDynCurrentLimitContinous(this->devHandler, motor, dparams.curContin);
	Dunker_setDynCurrentLimitTime(this->devHandler, motor, dparams.curTime);
	
	Dunker_setAcceleration(this->devHandler, motor, dparams.accV, dparams.accT);
	Dunker_setDecceleration(this->devHandler, motor, dparams.decV, dparams.decT);
	Dunker_configureErrorsAndFeedback(this->devHandler, motor);
	Dunker_setMaxPosFollowingError(this->devHandler, motor, dparams.maxPosErr);
	Dunker_enablePower(this->devHandler, motor);
	return true;
}

void Dunkermotoren::update() throw(MotorHandlerUnknownMotorException, MotorHandlerErrorWritingToPortException)
{
	bool isMotorMoving;
	int positions[motors.size()];
	int velocities[motors.size()];
	int status_words[motors.size()];
	int voltages[motors.size()];
	int kps[motors.size()];
	int kis[motors.size()];
	int kds[motors.size()];
	
	memset(kps,0,sizeof(int)*motors.size());
	memset(kis,0,sizeof(int)*motors.size());
	memset(kds,0,sizeof(int)*motors.size());
	
	getSyncPosition(motors.keys(), positions);
	getSyncVelocities(motors.keys(), velocities);
	getSyncStatusWords(motors.keys(), status_words);
	getSyncVoltages(motors.keys(), voltages);
	getSyncVelKps(motors.keys(), kps);
	getSyncVelKis(motors.keys(), kis);
	getSyncVelKds(motors.keys(), kds);

	int cnt=0;
// 	qDebug()<<"--------------------";
	foreach( Servo *s, motors)
	{
		Servo::TMotorData &data = s->data;
		if (data.status_word != status_words[cnt])
		{
			qDebug() << "status word: "<<data.status_word<<" "<<status_words[cnt];
// 			Dunker_printStatusWordInfo(data.status_word );
		}
		
// 		std::cout << "motor "<<s->data.name.toStdString()<<" at position " << positions[cnt]<<std::endl;
		//~ std::cout << "motor at velocity " << velocities[cnt]<<std::endl;
		//~ std::cout << "motor voltage " << voltages[cnt]<<std::endl;
		//~ std::cout << "motor kp " << kps[cnt]<<std::endl;
		//~ std::cout << "motor ki " << kis[cnt]<<std::endl;
		//~ std::cout << "motor kd " << kds[cnt]<<std::endl;
		if(voltages[cnt]<18000)
		{

			qDebug()<<"/================================================================\\";
			qDebug()<<"                LOW BATTERY LEVEL: "<<voltages[cnt]/1000.f<<"v";
			qDebug()<<"|================================================================|";
			qDebug()<<"\\________________________________________________________________/";
		}
		isMotorMoving=(velocities[cnt] != 0);
		memory_mutex->lock();
			data.antPosRads = data.currentPosRads;
			data.currentPos = positions[cnt]*((M_PIl*2.)/(1024.*72.));
			data.currentPosRads = positions[cnt]*((M_PIl*2.)/(1024.*72.));
			data.isMoving = isMotorMoving;
			data.status_word = status_words[cnt];
		memory_mutex->unlock();
		cnt++;
		
		if(Dunker_errorFlag(data.status_word))
		{
			qDebug() << "Dunkermotoren::Update(): ERROR An error has ocurred on motor "<<s->data.name;
			int aux_error_register;
			getErrorRegister((uchar)s->params.busId, aux_error_register);
			qDebug()<<"Dunkermotoren::Update(): Error register "<<aux_error_register;
			reset(data.busId);
		}
	}
}


//////////Abstract class implementation//////////////////

/******************************************************************************/
/* Send a position read command to AI-motor                                   */
/* Input : ServoID                                                            */
/* Output : Position                                                          */
/******************************************************************************/
void Dunkermotoren::getPosition(const QString &motor,float &fpos) throw(MotorHandlerUnknownMotorException,MotorHandlerErrorWritingToPortException)
{
	QMutexLocker locker(hardware_mutex);
	///Read current position
	int pos;
	if(motors.contains(motor))
	{
		if(Dunker_getPos(this->devHandler, motors[motor]->params.busId, &pos ) == 0)
		{
			fpos=pos; 
		}	
		else
		{
			qDebug()<<"DunkerMotoren::getPosition: Error getting pos from "<<motor;
		}
	}
	else
	{
		qDebug()<<"DunkerMotoren::getPosition: Error. "+motor+" doesn't exist on Dunkermotoren bus.";
	}
		
}

void Dunkermotoren::getSyncPosition(const QList<QString> &in_motors,int* pos) throw(MotorHandlerUnknownMotorException,MotorHandlerErrorWritingToPortException)
{
	QMutexLocker locker(hardware_mutex);
	uint8_t NodeIds[motors.size()];
	int k=0;

	foreach (QString motor_name, in_motors)
	{
		if(motors.contains(motor_name))
		{
			NodeIds[k] = motors[motor_name]->params.busId;
		}
		else
		{
			qDebug()<<"DunkerMotoren::getPosition: Error. "+motor_name+" doesn't exist on Dunkermotoren bus.";
		}
		k++;
	}

	if (Dunker_syncGetPosition(this->devHandler, motors.size(), NodeIds, pos) != 0)
	{
		qDebug()<<"DunkerMotoren::getSyncPosition: Error getting positions from motors";
	}
}

 

void Dunkermotoren::setPosition(const QString &motor,  float  pos, float maxSpeed) throw(MotorHandlerUnknownMotorException,MotorHandlerErrorWritingToPortException)
{
	QMutexLocker locker(hardware_mutex);
	if(motors.contains(motor))
	{
		qDebug() << "Dunkermotoren::setPosition(): disablingPower";
		Dunker_disablePower(this->devHandler, motors[motor]->params.busId);
		int ret=-1;
		//OBTENER LA POSICION
		qDebug() << "Dunkermotoren::setPosition(): Device "<<this->devHandler<<" motor "<<motors[motor]->params.busId<<" mode "<< motors[motor]->data.mode;
		if(motors[motor]->data.mode != MODE_POS)
		{
			printf("Dunkermotoren::setPosition(): Changing mode to MODE_POS\n");
			if((ret=Dunker_setOperationMode(this->devHandler, motors[motor]->params.busId, MODE_POS)) != 0)
			{
				//~ qDebug << "Dunkermotoren::setPosition() - Error changing mode (" << ((int)MODE_POS) << "): ";
				throw QString("Dunkermotoren::setPosition() - Error changing mode ("+QString::number(MODE_POS)+"): "+QString::number(ret));
			}
			else
			{
				motors[motor]->data.mode = MODE_POS;
			}
		}
		Dunker_openBreak(this->devHandler, motors[motor]->params.busId);
		Dunker_enablePower(this->devHandler, motors[motor]->params.busId);
		ret=-1;
		qDebug() << "Dunkermotoren::setPosition(): Device "<<this->devHandler<<" motor "<<motors[motor]->params.busId<<" lpos "<< pos;
		if((ret=Dunker_setVel(this->devHandler, motors[motor]->params.busId, motors[motor]->rads2Steps(motors[motor]->params.maxVelocity))) != 0)
			throw QString("Dunkermotoren::setPosition() - Error setting velocity ("+QString::number( motors[motor]->rads2Steps(motors[motor]->params.maxVelocity))+"): "+QString::number(ret));
		if((ret=Dunker_moveToAbsolutePosition(this->devHandler, motors[motor]->params.busId, motors[motor]->rads2Steps(pos))) != 0)
			throw QString("Dunkermotoren::setPosition() - Error setting position ("+QString::number(pos)+"): "+QString::number(ret));
	}
	else
	{
			qDebug()<<"DunkerMotoren::getPosition: Error. "+motor+" doesn't exist on Dunkermotoren bus.";
	}
}



	//~ k=0;
	//~ int ret;
	//~ foreach( Handler::GoalVelocity goal, goals)
	//~ {
		//~ if(motors[goal.name]->data.mode != MODE_VEL)
		//~ {
			//~ printf("Dunkermotoren::setSyncReferenceVelocity(): Changing mode to MODE_VEL\n");
			//~ 
			//~ if((ret=Dunker_setOperationMode(this->devHandler, motors[goal.name]->params.busId, MODE_VEL)) != 0)
			//~ {
				//~ qDebug << "Dunkermotoren::setSyncReferenceVelocity() - Error changing mode (" << ((int)MODE_POS) << "): ";
				//~ throw QString("Dunkermotoren::setSyncReferenceVelocity() - Error changing mode ("+QString::number(MODE_VEL)+"): "+QString::number(ret));
			//~ }
			//~ else
			//~ {
				//~ motors[goal.name]->data.mode = MODE_VEL;
			//~ }
			//~ 
		//~ }
	//~ 
		//~ qDebug()<<"Dunkermotoren::setSyncReferenceVelocity(): configuring motor"<<goal.busDir<<" to vel "<<motors[goal.name]->rads2Steps(goal.velocity);
		//~ NodeIds[k] = goal.busDir;
		//~ vels[k] = motors[goal.name]->rads2Steps(goal.velocity);
		//~ qDebug()<<"Dunkermotoren::setSyncReferenceVelocity(): configured motor"<<NodeIds[k]<<" to vel "<<vels[k];
		//~ k++;
	//~ }
	//~ if ( (ret=Dunker_syncSetVel(this->devHandler, numMotors, NodeIds, vels)) != 0 )
	//~ {
		//~ throw QString("Dynamixel::setSyncReferenceVelocity() - Error writing to port");
	//~ }
	//~ else
	//~ {
		//~ qDebug()<<"Dunkermotoren::setSyncReferenceVelocity(): vel. setted";
	//~ }


void Dunkermotoren::setSyncPosition( const QVector<Dunkermotoren::GoalPosition> & goals) throw(MotorHandlerErrorWritingToPortException)
{
	QMutexLocker locker(hardware_mutex);
	qDebug()<<"Dunkermotoren::setSyncPosition() (goals="<<goals.size()<<")";
	
	int numMotors = goals.size();
	int poss[numMotors];
	uint8_t NodeIds[numMotors];
	int ret;
	
	//~ 
	//~ for(k=0;k<goals.size();k++)
	//~ {
		//~ qDebug()<<"Dunkermotoren::setSyncPosition(): goals"<<goals[k].busDir<<" to pos (rad)"<<goals[k].position;
	//~ }
	//FIXME Es una forma muy rebuscada, pero necesitamos acceso a los distintos servos y no he visto ninguna más sencilla
	int k=0;
	foreach( Dunkermotoren::GoalPosition goal, goals)
	{
		if(motors.contains(goal.name))
		{
			if(motors[goal.name]->data.mode != MODE_POS)
			{
				qDebug()<<"Dunkermotoren::setSyncPosition(): Changing "<<QString(goal.name)<<" mode to MODE_POS\n";
				
				if((ret=Dunker_setOperationMode(this->devHandler, motors[goal.name]->params.busId, MODE_POS)) != 0)
				{
					qDebug() << "Dunkermotoren::setSyncPosition() - Error changing mode (" << ((int)MODE_POS) << "): ";
					throw QString("Dunkermotoren::setSyncPostion() - Error changing mode ("+QString::number(MODE_POS)+"): "+QString::number(ret));
				}
				else
				{
					motors[goal.name]->data.mode = MODE_POS;
				}
				
			}
		
			qDebug()<<"Dunkermotoren::setSyncPosition(): configuring motor"<<goals[k].busDir<<" to pos "<<motors[goal.name]->rads2Steps(goals[k].position);
			NodeIds[k] = goals[k].busDir;
			poss[k] = motors[goal.name]->rads2Steps(goals[k].position);
			qDebug()<<"Dunkermotoren::setSyncPosition(): configured motor"<<NodeIds[k]<<" to pos "<<poss[k];
		}
		else
		{
			qDebug()<<"DunkerMotoren::getPosition: Error. "+goal.name+" doesn't exist on Dunkermotoren bus.";
		}
		k++;
	}
	
	if ( (ret=Dunker_syncMoveToAbsolutePosition(this->devHandler, numMotors, NodeIds, poss)) != 0 )
	  throw QString("Dynamixel::setSyncPosition() - Error writing to port");
}

void Dunkermotoren::setSyncRelativePosition( const QVector<Dunkermotoren::GoalPosition> & goals) throw(MotorHandlerErrorWritingToPortException)
{
	qDebug()<<"Dunkermotoren::setSyncPosition() (goals="<<goals.size()<<")";
	QMutexLocker locker(hardware_mutex);
	
	int numMotors = goals.size();
	int poss[numMotors];
	uint8_t NodeIds[numMotors];
	
	int k=0;
	for(k=0;k<goals.size();k++)
	{
		qDebug()<<"Dunkermotoren::setSyncPosition(): goals"<<goals[k].busDir<<" to pos (rad)"<<goals[k].position;
	}
	//FIXME Es una forma muy rebuscada, pero necesitamos acceso a los distintos servos y no he visto ninguna más sencilla
	foreach( Servo *s, motors)
	{
		for(k=0;k<goals.size();k++)
		{
			if(s->params.busId == goals[k].busDir)
			{
				qDebug()<<"Dunkermotoren::setSyncPosition(): configuring motor"<<goals[k].busDir<<" to pos "<<s->rads2Steps(goals[k].position);
				NodeIds[k] = goals[k].busDir;
				poss[k] = s->rads2Steps(goals[k].position);
				qDebug()<<"Dunkermotoren::setSyncPosition(): configured motor"<<NodeIds[k]<<" to pos "<<poss[k];
			}
		}
	}
	int ret;
	if ( (ret=Dunker_syncMoveToRelativePosition(this->devHandler, numMotors, NodeIds, poss)) != 0 )
	  throw QString("Dynamixel::setSyncPosition() - Error writing to port");
}




bool Dunkermotoren::getVelocity( uchar motor, float & vel )
{
	QMutexLocker locker(hardware_mutex);
	int i_vel;
	if(Dunker_getVel(this->devHandler, motor, &i_vel ) == 0)
	{
		vel=i_vel;
		return true;
	}
	else
	{
		throw QString("Dunkermotoren::setSyncPosition() - Error getting Velocity");
		return false;
	}
}

void Dunkermotoren::getSyncVoltages(const QList<QString> &in_motors,int* voltages) throw(MotorHandlerUnknownMotorException,MotorHandlerErrorWritingToPortException)
{
	QMutexLocker locker(hardware_mutex);
	uint8_t NodeIds[motors.size()];
	int k=0;
	foreach(QString motor_name, in_motors)
	{
		NodeIds[k] = motors[motor_name]->params.busId;
		k++;
	}
	Dunker_syncGetMotorVoltages(this->devHandler, motors.size(), NodeIds, voltages);
}

void Dunkermotoren::getSyncVelKps(const QList<QString> &in_motors,int* kps) throw(MotorHandlerUnknownMotorException,MotorHandlerErrorWritingToPortException)
{
	QMutexLocker locker(hardware_mutex);
	uint8_t NodeIds[motors.size()];
	int k=0;
	foreach(QString motor_name, in_motors)
	{
		NodeIds[k] = motors[motor_name]->params.busId;
		k++;
	}
	Dunker_syncGetMotorVoltages(this->devHandler, motors.size(), NodeIds, kps);
}

void Dunkermotoren::getSyncVelKis(const QList<QString> &in_motors,int* kis) throw(MotorHandlerUnknownMotorException,MotorHandlerErrorWritingToPortException)
{
	QMutexLocker locker(hardware_mutex);
	uint8_t NodeIds[motors.size()];
	int k=0;
	foreach(QString motor_name, in_motors)
	{
		NodeIds[k] = motors[motor_name]->params.busId;
		k++;
	}
	Dunker_syncGetMotorVoltages(this->devHandler, motors.size(), NodeIds, kis);
}

void Dunkermotoren::getSyncVelKds(const QList<QString> &in_motors,int* kds) throw(MotorHandlerUnknownMotorException,MotorHandlerErrorWritingToPortException)
{
	QMutexLocker locker(hardware_mutex);
	uint8_t NodeIds[motors.size()];
	int k=0;
	foreach(QString motor_name, in_motors)
	{
		NodeIds[k] = motors[motor_name]->params.busId;
		k++;
	}
	Dunker_syncGetMotorVoltages(this->devHandler, motors.size(), NodeIds, kds);
}





void Dunkermotoren::getSyncVelocities(const QList<QString> &in_motors,int* vel) throw(MotorHandlerUnknownMotorException,MotorHandlerErrorWritingToPortException)
{
	QMutexLocker locker(hardware_mutex);
	uint8_t NodeIds[motors.size()];
	int k=0;
	foreach(QString motor_name, in_motors)
	{
		NodeIds[k] = motors[motor_name]->params.busId;
		k++;
	}
	Dunker_syncGetVelocity(this->devHandler, motors.size(), NodeIds, vel);
}


bool Dunkermotoren::setVelocity(const QString &motor, float  vel)
{
	QMutexLocker locker(hardware_mutex);

	motors[motor]->params.maxVelocity=vel;
		
	if(Dunker_setVel( this->devHandler, motors[motor]->params.busId, vel ) == 0)
	{
		return true;
	}
	else
	{	
		throw QString("Dunkermotoren::setVelocity() - Error setting Velocity");
		return false;
	}
}

bool Dunkermotoren::getReferenceVelocity(uchar  motor, float &vel)
{
	QMutexLocker locker(hardware_mutex);
	int i_vel;
	if(Dunker_getVel(this->devHandler, motor, &i_vel ) == 0)
	{
		vel=i_vel;
		return true;
	}
	else
	{
		throw QString("Dunkermotoren::getReferenceVelocity() - Error getting Velocity");
		return false;
	}
}

bool Dunkermotoren::setReferenceVelocity(const QString &motor, float vel)
{
	QMutexLocker locker(hardware_mutex);
	int ret=-1;
	qDebug() << "Dunkermotoren::setReferenceVelocity(): Device "<<this->devHandler<<" motor "<<motors[motor]->params.busId<<" mode "<< motors[motor]->data.mode;
	if (motors[motor]->data.mode!=MODE_VEL)
	{
		printf("Dunkermotoren::setReferenceVelocity() Changing mode to MODE_VEL\n");
		if((ret=Dunker_setOperationMode(this->devHandler, motors[motor]->params.busId, MODE_VEL)) != 0)
		{
			//~ qDebug << "Dunkermotoren::setReferenceVelocity() - Error changing mode (" << MODE_VEL << "): " <<ret;
			throw QString("Dunkermotoren::setReferenceVelocity() - Error changing mode ("+QString::number(vel)+"): "+QString::number(ret));
			return false;
		}
		else
		{
			Dunker_openBreak(this->devHandler, motors[motor]->params.busId);
			Dunker_enablePower(this->devHandler, motors[motor]->params.busId);
			motors[motor]->data.mode=MODE_VEL;
		}
	}
	
	ret = -1;
	qDebug() << "Device "<<this->devHandler<<" motor "<<motors[motor]->params.busId<<" vel "<< vel;
	if((ret=Dunker_setVel(this->devHandler, motors[motor]->params.busId, vel)) != 0)
	{
		//~ qDebug << "Dunkermotoren::setReferenceVelocity() - Error setting velocity (" << QString::number(vel) << "): "+QString::number(ret);
		throw QString("Dunkermotoren::setReferenceVelocity() - Error setting velocity ("+QString::number(vel)+"): "+QString::number(ret));
		return false;
	}
	return true;
}

bool Dunkermotoren::setSyncReferenceVelocity( const QVector<Dunkermotoren::GoalVelocity> & goals) throw(MotorHandlerErrorWritingToPortException)
{
	QMutexLocker locker(hardware_mutex);
	int numMotors = goals.size();
	int vels[numMotors];
	uint8_t NodeIds[numMotors];
	
	int k=0;
	for(k=0;k<goals.size();k++)
	{
		qDebug()<<"Dunkermotoren::setSyncReferenceVelocity(): goals"<<goals[k].name<<" to vel (rad)"<<goals[k].velocity;
		qDebug()<<"Dunkermotoren::setSyncReferenceVelocity(): goals"<<goals[k].name<<" to vel "<<motors[goals[k].name]->rads2Steps(goals[k].velocity);
	}
	
	//FIXME Es una forma muy rebuscada, pero necesitamos acceso a los distintos servos y no he visto ninguna más sencilla
	k=0;
	int ret;
	foreach( Dunkermotoren::GoalVelocity goal, goals)
	{
		if(motors[goal.name]->data.mode != MODE_VEL)
		{
			printf("Dunkermotoren::setSyncReferenceVelocity(): Changing mode to MODE_VEL\n");
			
			if((ret=Dunker_setOperationMode(this->devHandler, motors[goal.name]->params.busId, MODE_VEL)) != 0)
			{
				//~ qDebug << "Dunkermotoren::setSyncReferenceVelocity() - Error changing mode (" << ((int)MODE_POS) << "): ";
				throw QString("Dunkermotoren::setSyncReferenceVelocity() - Error changing mode ("+QString::number(MODE_VEL)+"): "+QString::number(ret));
			}
			else
			{
				motors[goal.name]->data.mode = MODE_VEL;
			}
			
		}
	
		qDebug()<<"Dunkermotoren::setSyncReferenceVelocity(): configuring motor"<<goal.busDir<<" to vel "<<motors[goal.name]->rads2Steps(goal.velocity);
		NodeIds[k] = goal.busDir;
		vels[k] = motors[goal.name]->rads2Steps(goal.velocity);
		qDebug()<<"Dunkermotoren::setSyncReferenceVelocity(): configured motor"<<NodeIds[k]<<" to vel "<<vels[k];
		k++;
	}
	//~ if ( (ret=Dunker_syncClearActualPosition(this->devHandler, numMotors, NodeIds)) != 0 )
	//~ {
		//~ throw QString("Dunkermotoren::syncClearActualPosition() - Error writing to port");
	//~ }
	//~ else
	//~ {
		//~ qDebug()<<"Dunkermotoren::syncClearActualPosition(): vel. setted";
	//~ }
	
	
	if ( (ret=Dunker_syncSetVel(this->devHandler, numMotors, NodeIds, vels)) != 0 )
	{
		throw QString("Dunkermotoren::setSyncReferenceVelocity() - Error writing to port");
	}
	else
	{
		qDebug()<<"Dunkermotoren::setSyncReferenceVelocity(): vel. setted";
	}
	return true;
}

bool Dunkermotoren::getMaxPosition( uchar motor, float & pos )
{
	QMutexLocker locker(hardware_mutex);
	int i_pos;
	if(Dunker_getMaxPos( this->devHandler, motor, &i_pos ) == 0)
	{
		pos=i_pos;
		return true;
	}
	else
		return false;
}

bool Dunkermotoren::getMinPosition( uchar motor, float & pos )
{
	QMutexLocker locker(hardware_mutex);
	int i_pos;
	if(Dunker_getMinPos( this->devHandler, motor, &i_pos ) == 0)
	{
		pos=i_pos;
		return true;
	}
	else
		return false;
}

bool Dunkermotoren::setMaxPosition(const QString &motor,float pos)
{
	QMutexLocker locker(hardware_mutex);
	if(Dunker_setMaxPos( this->devHandler, motors[motor]->params.busId, pos ) == 0)
		return true;
	else
		return false;
}

bool Dunkermotoren::setMinPosition(const QString &motor,float pos)
{
	QMutexLocker locker(hardware_mutex);
	if(Dunker_setMinPos( this->devHandler, motors[motor]->params.busId, pos) == 0)
		return true;
	else
		return false;
}

bool Dunkermotoren::powerOn( uchar motor )
{
	QMutexLocker locker(hardware_mutex);
	
	if(Dunker_enablePower( this->devHandler, motor ) == 0)
		return true;
	else
		return false;
	return false;
	
}

bool Dunkermotoren::powerOff( uchar motor )
{
	if(Dunker_disablePower( this->devHandler, motor ) == 0)
		return true;
	else
		return false;
	printf("Error: powerOn(): Not implemented yet\n");
	return false;
}

bool Dunkermotoren::getPower( uchar motor, float & pow )
{
	QMutexLocker locker(hardware_mutex);
	int i_pow;
	if(Dunker_getPower( this->devHandler, motor, &i_pow ) == 0)
	{
		pow=i_pow;
		return true;
	}
	else
		return false;
}

bool Dunkermotoren::getStatusWord( uchar motor, int & status_word )
{
	QMutexLocker locker(hardware_mutex);
	if(Dunker_getStatusWord( this->devHandler, motor, &status_word ) == 0)
	{
		return true;
	}
	else
		return false;
}

void Dunkermotoren::getSyncStatusWords(const QList<QString> &in_motors, int* status_words) throw(MotorHandlerUnknownMotorException,MotorHandlerErrorWritingToPortException)
{
	uint8_t NodeIds[motors.size()];
	int k=0;
	foreach(QString motor_name, in_motors)
	{
		NodeIds[k] = motors[motor_name]->params.busId;
		k++;
	}
	QMutexLocker locker(hardware_mutex);
	Dunker_syncGetStatusWord(this->devHandler, motors.size(), NodeIds, status_words);
}

bool Dunkermotoren::getErrorRegister( uchar motor, int & error_register )
{
	QMutexLocker locker(hardware_mutex);
	if(Dunker_getErrorRegister( this->devHandler, motor, &error_register ) == 0)
	{
		return true;
	}
	else
		return false;
}


bool Dunkermotoren::setId( uchar motor, int id )
{
	QMutexLocker locker(hardware_mutex);
	
	if(Dunker_changeNodeId( this->devHandler, motor, id)== 0)
	{
		return true;
	}
	else
		return false;
}

bool Dunkermotoren::setBaudrate( uchar motor, int baud )
{
	QMutexLocker locker(hardware_mutex);
	
	if(Dunker_setBaudRate( this->devHandler , baud) == 0)
		return true;
	else
		return false;
}

bool Dunkermotoren::getBaudrate(uchar motor, int &br)
{
	QMutexLocker locker(hardware_mutex);

	printf("Dunkermotoren::getBaudrate() Not implemented yet\n");
	return false;
}

/**
 * Sets Voltage Limit
 * @param motor 
 * @param limit 
 * @return 
 */
bool Dunkermotoren::setVoltageLimit( uchar motor, int limit )
{
	QMutexLocker locker(hardware_mutex);
	
	printf("Dunkermotoren::setVoltageLimit(): Not implemented yet\n");
	return false;
}

bool Dunkermotoren::getVoltageLimit(const QString &motor , int & limit )
{
	QMutexLocker locker(hardware_mutex);
	if(motors.contains(motor))
	{
		if( Dunker_getMaxMotorVoltage( this->devHandler , motors[motor]->params.busId, &limit ) == 0)
		{
			return true;
		}
		else
		{
			rDebug("DunkerMotoren::isMoving(): Error getting Voltage Limit  on "+motor);
			return false;
		}
	}
	else
	{
		rDebug("DunkerMotoren::getVoltageLimit(): Error. "+motor+" doesn't exist on Dunkermotoren bus.");
		return false;
	}
}

bool Dunkermotoren::isMoving(const QString &motor )
{
	QMutexLocker locker(hardware_mutex);
	int vel;
	if(motors.contains(motor))
	{
		if( Dunker_getVel( this->devHandler, motors[motor]->params.busId, &vel ) == 0)
		{
			if(vel != 0)
				return true;
			else
				return false;
		}
		else
		{
			rDebug("DunkerMotoren::isMoving(): Error getting vel "+motor);
			return false;
		}
	}
	else
	{
		rDebug("DunkerMotoren::getAcceleration(): Error. "+motor+" doesn't exist on Dunkermotoren bus.");
		return false;
	}
			
}

bool Dunkermotoren::getPunch(uchar motor, int & d) //Named PUNCH in Dunkermotoren manual
{
	QMutexLocker locker(hardware_mutex);
	printf("Error: getPunch(): Not implemented yet\n");
	return false;
}

bool Dunkermotoren::setPunch(uchar motor, int d)
{
	QMutexLocker locker(hardware_mutex);
	printf("Error: setPunch(): Not implemented yet\n");
	return false;
}

bool Dunkermotoren::setBothComplianceSlopes(uchar motor, int p) //Compliance slope in both rotation senses
{
	QMutexLocker locker(hardware_mutex);

	printf("Error: setBothComplianceSlopes(): Not implemented yet\n");
	return false;
}

bool Dunkermotoren::getCWComplianceMargin(uchar motor, int & db)
{
	QMutexLocker locker(hardware_mutex);
	printf("Error: getCWComplianceMargin(): Not implemented yet\n");
	return false;
}

bool Dunkermotoren::getCCWComplianceMargin(uchar motor, int & db)
{
	QMutexLocker locker(hardware_mutex);

	printf("Error: getCCWComplianceMargin(): Not implemented yet\n");
	return false;
}

bool Dunkermotoren::getCWComplianceSlope(uchar motor, int & db)
{
	QMutexLocker locker(hardware_mutex);

	printf("Error: getCWComplianceSlope(): Not implemented yet\n");
	return false;
}

bool Dunkermotoren::getCCWComplianceSlope(uchar motor, int & db)
{
	QMutexLocker locker(hardware_mutex);

	printf("Error: getCCWComplianceSlope(): Not implemented yet\n");
	return false;
}

bool Dunkermotoren::setReturnDelayTime( uchar motor, int t)
{
	QMutexLocker locker(hardware_mutex);
	
	printf("Error: setReturnDelayTime(): Not implemented yet\n");
	return false;
}

bool Dunkermotoren::getReturnDelayTime( uchar motor, int & t)
{
	QMutexLocker locker(hardware_mutex);

	printf("Error: getReturnDelayTime(): Not implemented yet\n");
	return false;
}

bool Dunkermotoren::getStatusReturnLevel(uchar motor,  int &level)
{
	printf("Error: getStatusReturnLevel(): Not implemented yet\n");
	return false;
}

bool Dunkermotoren::setStatusReturnLevel(uchar motor,  int &level)
{
	printf("Error: setStatusReturnLevel(): Not implemented yet\n");
	return false;
}

bool Dunkermotoren::getAcceleration(const QString &motor,   int & deltaV, int& deltaT )
{
	QMutexLocker locker(hardware_mutex);
	if(motors.contains(motor))
	{
		if( Dunker_getAcceleration( this->devHandler , motors[motor]->params.busId, &deltaV, &deltaT ) == 0)
		{
			return true;
		}
		else
		{
			rDebug("DunkerMotoren::getAcceleration(): Error getting aceleration "+motor);
			return false;
		}
	}
	else
	{
		rDebug("DunkerMotoren::getAcceleration(): Error. "+motor+" doesn't exist on Dunkermotoren bus.");
		return false;
	}
}


bool Dunkermotoren::getRevolutions(const QString &motor,  int& motorRevol, int& shaftRevol )
{
	QMutexLocker locker(hardware_mutex);
	if(motors.contains(motor))
	{
		if( Dunker_getMotorSaftRevolution( this->devHandler , motors[motor]->params.busId, &motorRevol ) == 0)
		{
			if( Dunker_getGearSaftRevolution( this->devHandler , motors[motor]->params.busId, &shaftRevol ) == 0)
			{
				return true;
			}
			else
			{
				rDebug("DunkerMotoren::getAcceleration(): Error getting gear saft revolutions "+motor);
				return false;
			}
		}
		else
		{
			rDebug("DunkerMotoren::getAcceleration(): Error getting motor saft revolution "+motor);
			return false;
		}
	}
	else
	{
		rDebug("DunkerMotoren::setZeroPos(): Error. "+motor+" doesn't exist on Dunkermotoren bus.");
		return false;
	}
}


void Dunkermotoren::setZeroPos(const QString &motor)
{
	QMutexLocker locker(hardware_mutex);
	if(motors.contains(motor))
	{
		Dunker_clearActualPosition(this->devHandler, motors[motor]->params.busId);
	}
	else
	{
		rDebug("DunkerMotoren::setZeroPos(): Error. "+motor+" doesn't exist on Dunkermotoren bus.");
	}
	
}

void Dunkermotoren::setSyncZeroPos()
{
	QMutexLocker locker(hardware_mutex);
	int numMotors = motors.size();
	uint8_t NodeIds[numMotors];
	int k=0;
	foreach( Servo *s, motors)
	{
				NodeIds[k] = s->data.busId;
				k++;
	}
	Dunker_syncClearActualPosition(this->devHandler, numMotors,NodeIds);
}

