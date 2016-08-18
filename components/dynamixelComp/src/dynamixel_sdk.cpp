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
#if COMPILE_DYNAMIXEL==1

#include "dynamixel_sdk.h"

/**
* \brief Default constructor
* @param busParams Struct contains bus params
* @param params Struct contains motor list params
* @param m Mutex shared with worker class
*/
DynamixelSDK::DynamixelSDK(  RoboCompJointMotor::BusParams  *busParams , RoboCompJointMotor::MotorParamsList *params, QMutex *m)
{
	h_mutex = new QMutex();
	m_mutex = new QMutex();
	this->busParams = busParams;
	this->params = params;
	w_mutex = m;
}
/**
* \brief Default destructor
*/
DynamixelSDK::~DynamixelSDK()
{
}
/**
 * \brief INITIALIZE METHOD
 */ 
void DynamixelSDK::initialize() throw (QString)
{	
	//check device string to transform into integer
	int deviceIndex = 0;
	switch ( "/dev/ttyUSB0")
	{
		case "/dev/ttyUSB0":
			deviceIndex = 0;
			break;
		case "/dev/ttyUSB1":
			deviceIndex = 1;
			break;
		default:
			//NOTE if dynamixel is not connected the program failed
			qFatal("|| ERROR In Dynamixel_sdk initialize:   DEVICE NOT IN ttyUSB0 OR ttyUSB1!!   ||");
			break;
	}
// 	if(busParams->device == "/dev/ttyUSB0")       deviceIndex = 0;
// 	else if(busParams->device == "/dev/ttyUSB1")  deviceIndex = 1;
// 	else qFatal("|| ERROR In Dynamixel_sdk initialize:   DEVICE NOT IN ttyUSB0 OR ttyUSB1!!   ||"); //NOTE if dynamixel is not connected the program failed
	 qDebug()<<"|| DEVICE NAME: "<<QString::fromStdString(busParams->device)<<"      DEVICE INDEX: "<<deviceIndex<<"  ||";
	
///FIXME Insert all posibles baudRate values	
	int baudnum = 1;
	switch ( busParams->baudRate )
	{
		case 1000000: baudnum = 1 ; break;
		default: baudnum = 1 ; break;
	}
	
	if( dxl_initialize(deviceIndex, baudnum) == 0 )
	{
		QString error;
		error = "JointMotor::Dynamixel::initialize() - Port " + QString::fromStdString( busParams->device ) +
			  " could not be opened. Please check file permissions";
		throw error;
	}

	//Create servos instances in a QMap indexed by name
	for (int i = 0; i < busParams->numMotors; i++)
	{
		std::cout << "JointMotor::Dynamixel::Dynamixel - name " <<params->operator[](i).name<< std::endl;
		QString name = QString::fromStdString(params->operator[](i).name);
		motors[name] = new Servo( params->operator[](i) );
		//steps_range , max_degrees, steps_speed_range, max_speed_rads  ¿¿Deberían venir en los parámetros?
		motors[name]->setMotorRanges(params->operator[](i).stepsRange,params->operator[](i).maxDegrees, 1024, params->operator[](i).maxVelocity);
		
		motorsId[name] = params->operator[](i).busId;
	}

	std::cout << "JointMotor::Dynamixel::Dynamixel - Motor Map created with " << busParams->numMotors << " motors: " << std::endl;
	foreach( Servo * s, motors.values())
	{
		std::cout << "	" + s->params.name << std::endl;
	}

	//Initialize motor params
	foreach( Servo *s, motors.values() )
	{
		Servo::TMotorData &data = s->data;
		RoboCompJointMotor::MotorParams &params = s->params;

		std::cout << "JointMotor::Dynamixel::Dynamixel - Configuration data of motor " << params.name << std::endl;
		///Set Status return level to 1. Level 0: no response ; Level 1: only for reading commands. Default ; Level 2: always
		int level = 1;
		setStatusReturnLevel(data.name, level);
		getStatusReturnLevel(data.name, level);
		qDebug() << "	Status return level: " << level;

		//set specific parameters
	    setPunch(data.name, 32);
		setBothComplianceMargins(data.name, 1);
		setBothComplianceSlopes(data.name, 20);

		///Return delay time
		int rt = 50;
		if (setReturnDelayTime( data.name, rt) == true and getReturnDelayTime( data.name, rt) == true)
		{
			qDebug() << "	Return delay time: " << rt;
		}
		else
			qDebug() << "Error setting delay time";

		///Control params
		int m;
		if (getPunch( data.name, m ) == true)
		{
			qDebug() << "	Punch: " << m;
		}
		else
			qDebug() << "Error reading Punch";

		if (getCCWComplianceMargin( data.name, m ) == true)
		{
			qDebug() << "	CCWComplianceMargin: " << m;
		}
		else
			qDebug() << "Error reading CCWComplianceMargin";

		if (getCWComplianceMargin( data.name, m ) == true)
		{
			qDebug() << "	CWComplianceMargin: " << m;
		}
		else
			qDebug() << "Error reading CWComplianceMargin";
		if (getCCWComplianceSlope( data.name, m ) == true)
		{
			qDebug() << "	CCWComplianceSlope: " << m;
		}
		else
			qDebug() << "Error reading CCWComplianceSlope";

		if (getCWComplianceSlope( data.name, m ) == true)
		{
			qDebug() << "	CWComplianceSlope: " << m;
		}
		else
			qDebug() << "Error reading CWComplianceSlope";

		///Read current position
		float p;
		getPosition(data.name, p );
		data.currentPosRads  = p;
		data.antPosRads = p;
		data.currentPos = s->rads2Steps(p);
		qDebug() << "	Current position (steps): " << data.currentPos;
		qDebug() << "	Current position (rads): " << data.currentPosRads;

		///Set limits
		qDebug()<<"inverted"<<params.invertedSign;
		if(params.invertedSign == true)
		{
			if( setMaxPosition(data.name, params.minPos) != true)
				qDebug()<<" Error setting min Position "<<params.minPos; 
			if( setMinPosition(data.name, params.maxPos) != true)
				qDebug()<<" Error setting max Position "<<params.maxPos;
		}
		else
		{
			if( setMaxPosition(data.name, params.maxPos)!= true)
				qDebug()<<" Error setting min Position "<<params.minPos; 
			if( setMinPosition(data.name, params.minPos) != true)
				qDebug()<<" Error setting max Position "<<params.maxPos;
		}
		
		getMaxPosition( data.name, p);
		qDebug() << "	Max position (nominal/steps/rads): " << params.maxPos << s->steps2Rads(p);
		getMinPosition( data.name, p);
		qDebug() << "	Min position (nominal/steps/rads): " << params.minPos << s->steps2Rads(p);

		///set servos to maximum speed
		data.maxVelocityRads = params.maxVelocity;
		setVelocity( data.name, params.maxVelocity);

		qDebug() << "	Max velocity " << params.maxVelocity;
	}
	
}

void DynamixelSDK::update() throw(MotorHandlerUnknownMotorException,MotorHandlerErrorWritingToPortException)
{
	float  currentPosRads;
	foreach( Servo *s, motors.values())
	{
		Servo::TMotorData &data = s->data;
		try
		{
			int temperature;
			getTemperature(data.name,temperature);
			getPosition( data.name, currentPosRads );
			w_mutex->lock();
			data.currentPosRads = currentPosRads;
			data.isMoving = fabs(data.antPosRads - data.currentPosRads) > 0.01;
			data.antPosRads = data.currentPosRads;
			data.currentPos = s->rads2Steps(data.currentPosRads);
			data.temperature = temperature;
			w_mutex->unlock();
		}
		catch(const MotorHandlerUnknownMotorException &ex)
		{
			throw ex;
		}
		catch(const MotorHandlerErrorWritingToPortException &ex)
		{
			throw ex;
		}
	}
}
///////////Private methods/////////////

/*

void Dynamixel::ping( char motor )
{
	packet[2]=motor;
	packet[3]=0x02;
	packet[4]=PING;
	packet[5]=checkSum(packet);
	sendIPacket(packet, 6);
	qDebug()<<"Pinging motor " << motor;
	getSPacket( );

	if( status[4] == 0 ) qDebug() << "Ping OK from motor " << status[2];
	else qDebug() << "Ping error from motor " << status[2];
}
*/

//////////Abstract class implementation//////////////////
/**
* \brief Send a position read command to AI-motor 
* @param motor
* @param pos 
*/
void DynamixelSDK::getPosition(const QString &motor, float  &pos) throw(MotorHandlerUnknownMotorException,MotorHandlerErrorWritingToPortException)
{	
	h_mutex->lock();
		int posInt = dxl_read_word( motorsId[motor], GET_PRESENT_POSITION_L );
	h_mutex->unlock();
		
	if(checkResponse())
		pos = motors[motor]->steps2Rads( posInt );
}
/**
* \brief Check sdk command response
* @return bool Return true if command executed sucessfully, false otherwise 
*/
bool DynamixelSDK::checkResponse()
{
	h_mutex->lock();
		int CommStatus = dxl_get_result();
	h_mutex->unlock();
	/*
	if( CommStatus != COMM_RXSUCCESS )
	{
		switch(CommStatus){
			case COMM_TXFAIL:
				printf("COMM_TXFAIL: Failed transmit instruction packet!\n");
				break;
			case COMM_TXERROR:
				printf("COMM_TXERROR: Incorrect instruction packet!\n");
				break;
			case COMM_RXFAIL:
				printf("COMM_RXFAIL: Failed get status packet from device!\n");
				break;
			case COMM_RXWAITING:
				printf("COMM_RXWAITING: Now recieving status packet!\n");
				break;
			case COMM_RXTIMEOUT:
				printf("COMM_RXTIMEOUT: There is no status packet!\n");
				break;
			case COMM_RXCORRUPT:
				printf("COMM_RXCORRUPT: Incorrect status packet!\n");
				break;
			default:
				printf("This is unknown error code!\n");
				break;
			}
		return false;
	}*/
	///TODO It's necessary check this method, some requests haven't got response
	return true;	
}

void DynamixelSDK::setPosition( const QString &motor,  float  pos, float maxSpeed ) throw(MotorHandlerUnknownMotorException,MotorHandlerErrorWritingToPortException)
{
	if( motorsId.contains( motor ) )
	{
		Servo *servo = this->motors[motor];

		if(maxSpeed < -servo->data.maxVelocityRads)
			setVelocity(motor,-servo->data.maxVelocityRads);
		else
			if(maxSpeed > servo->data.maxVelocityRads)
				setVelocity(motor,servo->data.maxVelocityRads);
			else
				setVelocity(motor, maxSpeed);
			
	
		int pInt = this->motors[motor]->rads2Steps(pos);
//	  qDebug()<<"setting position"<<pInt<<"of motor"<<motor<<motors[motor]->params.busId<<pos;

		h_mutex->lock();
			dxl_write_word( motorsId[motor], SET_GOAL_POSITION_L, pInt );  
		h_mutex->unlock();
	  
		if ( !checkResponse() )
		{
			throw MotorHandlerErrorWritingToPortException("DynamixelSDK::setPosition");
		}
	}
	else
	{
		rError("Set position error, unknow motor:" +motor);
		throw MotorHandlerUnknownMotorException( motor.toStdString());
	}
	qDebug()<<"done:setting position of motor"<<motor;
}

void DynamixelSDK::setSyncPosition( const QVector<Handler::GoalPosition> & goals) throw( MotorHandlerErrorWritingToPortException )
{
	int numMotors = goals.size();
	h_mutex->lock();
	
		// Make syncwrite velocity
		dxl_set_txpacket_id(BROADCAST_ID);
		dxl_set_txpacket_instruction(INST_SYNC_WRITE);
		dxl_set_txpacket_parameter(0, SET_VELOCITY);
		dxl_set_txpacket_parameter(1, 2);
		for(int i=0; i < numMotors; i++ )
		{
			int velocity = this->motors[goals[i].name]->radsPerSec2Steps(goals[i].maxVel);
qDebug()<<"motor"<<goals[i].name<<velocity<<goals[i].maxVel<<"pos"<<goals[i].position;
			dxl_set_txpacket_parameter(2+3*i, motorsId[goals[i].name]);
			dxl_set_txpacket_parameter(2+3*i+1, dxl_get_lowbyte(velocity));
			dxl_set_txpacket_parameter(2+3*i+2, dxl_get_highbyte(velocity));
		}
		dxl_set_txpacket_length((2+1)*numMotors+4);
		dxl_txrx_packet();
	
		// Make syncwrite position 
		dxl_set_txpacket_id(BROADCAST_ID);
		dxl_set_txpacket_instruction(INST_SYNC_WRITE);
		dxl_set_txpacket_parameter(0, SET_GOAL_POSITION_L);
		dxl_set_txpacket_parameter(1, 2);
		for(int i=0; i < numMotors; i++ )
		{
			int position = this->motors[goals[i].name]->rads2Steps(goals[i].position);
			dxl_set_txpacket_parameter(2+3*i, motorsId[goals[i].name]);
			dxl_set_txpacket_parameter(2+3*i+1, dxl_get_lowbyte(position));
			dxl_set_txpacket_parameter(2+3*i+2, dxl_get_highbyte(position));
		}
		dxl_set_txpacket_length((2+1)*numMotors+4);
		dxl_txrx_packet();
	h_mutex->unlock();
	
	if ( !checkResponse() )
	{
	  throw MotorHandlerErrorWritingToPortException("DynamixelSDK::setSyncPosition");
	}
}

bool DynamixelSDK::getVelocity(const QString &motor, float  & vel )
{
	h_mutex->lock();
		int v = dxl_read_word(motorsId[motor],GET_VELOCITY);
	h_mutex->unlock();

	if ( !checkResponse() )
		return false;

	vel = motors[motor]->radsPerSec2Steps(v);
	return true;
}

bool DynamixelSDK::setVelocity( const QString &motor, float  vel )
{
	int v = motors[motor]->radsPerSec2Steps(vel);
	h_mutex->lock();
		dxl_write_word(motorsId[motor],SET_VELOCITY ,v );
	h_mutex->unlock();

	if ( !checkResponse() )
		return false;

	motors[motor]->data.currentVelocityRads = vel;
	return true;
}

bool DynamixelSDK::getMaxPosition(const QString &motor, float  & pos )
{
	h_mutex->lock();
		pos = dxl_read_word(motorsId[motor],MAX_POSITION );
	h_mutex->unlock();

	if ( !checkResponse() )
		return false;

	pos = motors[motor]->steps2Rads( pos );
	return true;
}

bool DynamixelSDK::getMinPosition(const QString &motor, float  & pos )
{
	h_mutex->lock();
		pos = dxl_read_word(motorsId[motor],MIN_POSITION );
	h_mutex->unlock();

	if ( !checkResponse() )
		return false;

	pos = motors[motor]->steps2Rads( pos );
	return true;
}

bool DynamixelSDK::setMaxPosition( const QString &motor, float  pos )
{
	int pInt = this->motors[motor]->rads2Steps(pos);
	h_mutex->lock();
		dxl_write_word(motorsId[motor],MAX_POSITION,pInt );
	h_mutex->unlock();

	if ( !checkResponse() )
		return false;
	return true;
}

bool DynamixelSDK::setMinPosition( const QString &motor, float  pos )
{
	int pInt = this->motors[motor]->rads2Steps(pos);
	h_mutex->lock();
		dxl_write_word(motorsId[motor],MIN_POSITION,pInt );
	h_mutex->unlock();

	if ( !checkResponse() )
		return false;
	return true;
}

bool DynamixelSDK::powerOn( const QString &motor )
{
	h_mutex->lock();
		dxl_write_word(motorsId[motor],SET_POWER ,1 );
	h_mutex->unlock();

	if ( !checkResponse() )
		return false;

	return true;
}

bool DynamixelSDK::powerOff( const QString &motor )
{
	h_mutex->lock();
		dxl_write_word(motorsId[motor],SET_POWER ,0 );
	h_mutex->unlock();

	if ( !checkResponse() )
		return false;

	return true;
}

bool DynamixelSDK::getPower(const QString &motor, float & pow )
{
	h_mutex->lock();
		pow = dxl_read_word(motorsId[motor],PRESENT_LOAD );
	h_mutex->unlock();
	
	if ( !checkResponse() )
		return false;

	return true;
}

bool DynamixelSDK::setId(const QString &motor, int id )
{
	h_mutex->lock();
		dxl_write_word(motorsId[motor],ID_REGISTER,id );
	h_mutex->unlock();
	
	if ( !checkResponse() )
		return false;

	return true;
}

bool DynamixelSDK::setBaudrate(const QString &motor, int baud )
{
	h_mutex->lock();
		dxl_write_word(motorsId[motor],BAUD_RATE,baud );
	h_mutex->unlock();
	
	if ( !checkResponse() )
		return false;

	return true;
}

bool DynamixelSDK::getBaudrate(const QString &motor, int &br)
{
	h_mutex->lock();
		br = dxl_read_word(motorsId[motor],BAUD_RATE );
	h_mutex->unlock();
	
	if ( !checkResponse() )
		return false;

	return true;
}

/**
 * Sets Voltage Limit
 * @param motor
 * @param limit
 * @return
 */

bool DynamixelSDK::setVoltageLimit(const QString &motor, int limit )
{
	h_mutex->lock();
		dxl_write_word(motorsId[motor],VOLTAGE_LIMIT,limit );
	h_mutex->unlock();
	
	if ( !checkResponse() )
		return false;

	return true;
}

bool DynamixelSDK::getVoltageLimit(const QString &motor, int & limit )
{
	h_mutex->lock();
		limit = dxl_read_word(motorsId[motor],VOLTAGE_LIMIT );
	h_mutex->unlock();
	
	if ( !checkResponse() )
		return false;

	return true;
}

bool DynamixelSDK::isMoving(const QString &motor )
{
	h_mutex->lock();
		bool moving = dxl_read_word(motorsId[motor],MOVING );
	h_mutex->unlock();
	
	if ( !checkResponse() )
		return false;	//AMBIGUO

	return moving;
}

bool DynamixelSDK::getPunch(const QString &motor, int & d) //Named PUNCH in dynamixel manual
{
	h_mutex->lock();
		d = dxl_read_word(motorsId[motor],PUNCH );
	h_mutex->unlock();

	if ( !checkResponse() )
		return false;

	return true;
}

bool DynamixelSDK::setPunch(const QString &motor, int d)
{
	h_mutex->lock();
		dxl_write_word(motorsId[motor],PUNCH,d );
	h_mutex->unlock();

	if ( !checkResponse() )
		return false;

	return true;
}

bool DynamixelSDK::setBothComplianceSlopes(const QString &motor, int p) //Compliance slope in both rotation senses
{
	if ( p<0 or p> 255 )
	{
		qWarning("Compliance parameter out of range:%d. Try with a 0:255 value",p);
		return false;
	}
	
	h_mutex->lock();
		dxl_write_word(motorsId[motor],SET_CW_COMPLIANCE_SLOPE,p );
		dxl_write_word(motorsId[motor],SET_CCW_COMPLIANCE_SLOPE,p );
	h_mutex->unlock();
	
	if ( !checkResponse() )
		return false;

	return true;
}

bool DynamixelSDK::setBothComplianceMargins(const QString &motor, int db)
{
	h_mutex->lock();
		dxl_write_word(motorsId[motor],SET_CW_COMPLIANCE_MARGIN, db );
		dxl_write_word(motorsId[motor],SET_CCW_COMPLIANCE_MARGIN, db );
	h_mutex->unlock();
	
	if ( !checkResponse() )
		return false;

	return true;
}

bool DynamixelSDK::getCWComplianceMargin(const QString &motor, int &db)
{
	h_mutex->lock();
		db = dxl_read_word(motorsId[motor],SET_CW_COMPLIANCE_MARGIN );
	h_mutex->unlock();
	
	if ( !checkResponse() )
		return false;

	return true;
}

bool DynamixelSDK::getCCWComplianceMargin(const QString &motor, int &db)
{
	h_mutex->lock();
		db = dxl_read_word(motorsId[motor],SET_CCW_COMPLIANCE_MARGIN );
	h_mutex->unlock();
	
	if ( !checkResponse() )
		return false;

	return true;
}

bool DynamixelSDK::getCWComplianceSlope(const QString &motor, int &db)
{
	h_mutex->lock();
		db = dxl_read_word(motorsId[motor],SET_CW_COMPLIANCE_SLOPE );
	h_mutex->unlock();
	
	if ( !checkResponse() )
		return false;

	return true;
}

bool DynamixelSDK::getCCWComplianceSlope(const QString &motor, int &db)
{
	h_mutex->lock();
		db = dxl_read_word(motorsId[motor],SET_CCW_COMPLIANCE_SLOPE );
	h_mutex->unlock();
	
	if ( !checkResponse() )
		return false;

	return true;
}

bool DynamixelSDK::setReturnDelayTime(const QString &motor, int t)
{
	h_mutex->lock();
		dxl_write_word(motorsId[motor],RETURN_DELAY_TIME, t );
	h_mutex->unlock();
	
	if ( !checkResponse() )
		return false;

	return true;
}

bool DynamixelSDK::getReturnDelayTime(const QString &motor, int & t)
{
	h_mutex->lock();
		t = dxl_read_word(motorsId[motor],RETURN_DELAY_TIME );
	h_mutex->unlock();
	
	if ( !checkResponse() )
		return false;

	return true;
}

bool DynamixelSDK::getStatusReturnLevel(const QString &motor,  int &level)
{
	h_mutex->lock();
		level = dxl_read_word(motorsId[motor],STATUS_RETURN_VALUE );
	h_mutex->unlock();

	if ( !checkResponse() )
		return false;

	return true;
}

bool DynamixelSDK::setStatusReturnLevel(const QString &motor,  int level)
{
	h_mutex->lock();
		dxl_write_word(motorsId[motor],STATUS_RETURN_VALUE,level );
	h_mutex->unlock();

	if ( !checkResponse() )
		return false;

	return true;
}

bool DynamixelSDK::getTemperature(const QString &motor,int &temperature)
{
	h_mutex->lock();
		temperature = dxl_read_word(motorsId[motor],GET_TEMPERATURE );
	h_mutex->unlock();

	if ( !checkResponse() )
		return false;

	return true;
}

#endif
