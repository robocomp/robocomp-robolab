#include "robexhandler.h"
/**
 * \brief Default constructor
 * @param TMechParams Some mechanical params read form config file
*/
RobexHandler::RobexHandler(RoboCompDifferentialRobot::TMechParams _params): mechParams(_params)
{
	// Open and initialize the device
	MotionDevice.setName (QString::fromStdString(mechParams.device) );
	MotionDevice.setBaudRate ( QSerialPort::BAUD19200);

	if ( MotionDevice.open ( QIODevice::ReadWrite ) == false ){
		rError("Failed to open: "+QString::fromStdString(mechParams.device));

		if (mechParams.device == "/dev/ttyUSB0") mechParams.device = "/dev/ttyUSB1";
		else mechParams.device = "/dev/ttyUSB0";

		MotionDevice.setName(QString::fromStdString(mechParams.device));
		rDebug("Trying "+QString::fromStdString(mechParams.device));
		if (MotionDevice.open ( QIODevice::ReadWrite ) == false){
			rError("Fatal error, failed to open: "+QString::fromStdString(mechParams.device));
		}
	}
	rInfo("Device "+QString::fromStdString(mechParams.device)+" is open");
	//Read base mechanical data
	if(!getBaseData())
        {
		rError("Fatal error, error reading mechanical parameters");
                qFatal("Fatal error, error reading mechanical parameters");
        }
	//Initialize 
	executeSpeed(0.f,0.f);
	bState.x = bState.correctedX = 0;
	bState.z = bState.correctedZ = 0;
	bState.alpha = bState.correctedAlpha = 0;
	setMotorsState ( MOTORS_STATE_NORMAL );
	rInfo("Setting the motors state to 'normal operation' ...");
	
	commandToSend = false;
	adv = 0.f;
	rot = 0.f;
	LastCmdTimestamp = QTime::currentTime();
	bstate_mutex = new QMutex();
	speed_mutex = new QMutex();
	
	// Acceleration: 010 010
	char accRet[10];
	//sendCommand("D5:0100105*", &accRet[0], 3);
	sendCommand("D5:0040045*", &accRet[0], 3);
	
	accRet[3] = '\0';
	printf("Acceleration command result: %s\n", accRet);
}
/**
 * \brief Destructor
 */
RobexHandler::~RobexHandler(){
	executeSpeed(0.f,0.f);
	rInfo("Setting the motors state to 'free' ...");
	if (! setMotorsState ( MOTORS_STATE_FREE ) )
		rError("Error setting motors state! Do it manually." );
}
/**
* \brief
*/
void RobexHandler::compute()
{
	computePosBase();
	//computeAutonomy();
	if(commandToSend)
	{
		speed_mutex->lock();
			advE = adv;
			rotE = rot;
		speed_mutex->unlock();
		result = executeSpeed(advE,rotE);
		speed_mutex->lock();
			commandToSend = false;
		speed_mutex->unlock();
	}
	else
		checkMotorsState();
}

/**
* \brief Sends speed command to microcontroller
* @param adv Advance speed in mm/seg
* @param rot Turn speed in rad/seg
* @return true if command was sended successfully, else return false
*/
bool RobexHandler::executeSpeed(float adv,float rot)
{
	return setSpeedWheels ( ( ( adv- ( rot*mechParams.axisLength ) /2. ) /mechParams.wheelRadius ), ( ( adv+ ( rot*mechParams.axisLength ) /2. ) /mechParams.wheelRadius ) );
}

//Funciones accesibles desde el exterior

/**
 * \brief Get mechanical params
 * @return TMechParams current base mechanical params
 */
RoboCompDifferentialRobot::TMechParams RobexHandler::getMechParams()
{
	return mechParams;
}

/**
 * \brief Reset odometer
 * @return true if command was sended successfully, else return false
 */
bool RobexHandler::resetOdometer()
{
	if (setSpeedBase(0.f, 0.f))
	{
		rInfo("Odometer reset OK"); 
		bstate_mutex->lock();
		bState.x     = bState.correctedX     = 0;
		bState.z     = bState.correctedZ     = 0;
		bState.alpha = bState.correctedAlpha = 0.;
		bstate_mutex->unlock();
		return true;
	}

	rError("Odometer reset fail");
	return false;
}
/**
* \brief Set odometer to a specified state
* @param _bState State to set odometer values
* @return true if command was sended successfully, else return false
*/
bool RobexHandler::setOdometer(RoboCompGenericBase::TBaseState _bState)
{
	if (setSpeedBase (0.f, 0.f))
	{
		rInfo("Set odometer OK"); 
		bstate_mutex->lock();
		bState.x     = bState.correctedX     =_bState.x;
		bState.z     = bState.correctedZ     = _bState.z;
		bState.alpha = bState.correctedAlpha = _bState.alpha;
		bstate_mutex->unlock();
		return true;	
	}
	else
		rError("Set odometer fail");
	return false;
}
/**
 * \brief Return current base state
 * @return bState Struct contains base position compute by odometry
 */
RoboCompGenericBase::TBaseState RobexHandler::getBaseState()
{
	return bState;
}


/**
 * \brief Set the base speed in rads/sg
 * @param adv New base speed en mm/sg
 * @param rot New base rot speed en rads/sg
 * @return true if command executed successfully, else return false
 */
bool RobexHandler::setSpeedBase ( float _adv , float _rot ){
	speed_mutex->lock();
		adv = _adv;
		rot = _rot;
		commandToSend = true;
	speed_mutex->unlock();
/*	QTime t = QTime::currentTime();
	while (true)
	{
		if (t.elapsed() > 100)
			return false;
		speed_mutex->lock();
		if(commandToSend == false)
		{
			speed_mutex->unlock();
			return result;
		}
		speed_mutex->unlock();
		usleep(20);
	}*/
	return false;
}

/**
 * \brief Stops base
 * @return true if sucess
 */
bool RobexHandler::stopBase()
{
	return setSpeedBase(0.f,0.f);
}


//Internal

/**
 * \brief Read mechanical data from Robot. Private method for initialization from base microcontroller
 * @return true if data read successfully else return false
 */
bool RobexHandler::getBaseData ( )
{
	char buf[128];
	int i=0;
	bool success=false;

	QString cmd = QString ( BASEDATA_GET_COMMAND_STRING BASEDATA_GET_COMMAND_CRC );
	if ( sendCommand ( cmd, buf, BASEDATA_DATA_SIZE ) ){
		// Read the received data
		if ( buf[i] == 'D' && buf[++i] == '0' && buf[++i] == ':' ){
			i++;
			ascii2dec ( buf+i,BASEDATA_GETGEAR_DATASIZE,mechParams.gearRatio );
			i += BASEDATA_GETGEAR_DATASIZE +1;  // : added
			ascii2dec ( buf+i,BASEDATA_GETRADIO_DATASIZE,mechParams.wheelRadius );
			i += BASEDATA_GETRADIO_DATASIZE +1;  // : added
			ascii2dec ( buf+i,BASEDATA_GETBASELINE_DATASIZE,mechParams.axisLength );
//mechParams.axisLength = 413;
	
	mechParams.encoderSteps = ENCODER_STEPS;  //OJO hay que leerlo de micro
	    	mechParams.temp = ( M_PI * mechParams.wheelRadius ) / ( mechParams.gearRatio*mechParams.encoderSteps ); 

			rInfo("READING MECHANICAL PARAMETERS FROM ROBOT");
			rInfo("Gear Ratio: "+QString::number(mechParams.gearRatio));
			rInfo("Wheel Radius: "+QString::number(mechParams.wheelRadius));
			rInfo("Axis Length: "+QString::number(mechParams.axisLength));
			rInfo("Encoder steps: "+QString::number(mechParams.encoderSteps));
			success=true;
		}
	}
	return success;
}

/**
 * \brief Read wheels position as measured by encoders
 * @param left Left wheel position
 * @param right Right wheel position 
 * @return true if position read successfully, else return false
 */
bool RobexHandler::getPosWheels ( int &left, int &right )
{
	static char buf[128];
	int *aux;
	QString cmd;
	uchar temp[4];
	bool success=false;

	cmd = QString ( ODOMETER_GETPOSWHEELS_B_STRING ODOMETER_GETPOSWHEELS_B_CRC );
	if (sendCommand(cmd, buf, 13))
	{
		// Read the received data
		//Check control chars
		if ( buf[0] == 'D' && buf[1] == '4' && buf[2] == ':' )
		{
			temp[0] = buf[3];
			temp[1] = buf[4];
			temp[2] = buf[5];
			temp[3] = buf[6];
			aux = ( int* ) temp;
			right = *aux;
			temp[0] = buf[7];
			temp[1] = buf[8];
			temp[2] = buf[9];
			temp[3] = buf[10];
		
			aux = ( int* ) temp;
			left = *aux;	
			success=true;
		}
	}
	return success;
}


void RobexHandler::computeAutonomy()
{
	static int tick = 0;
	if (tick % 100 == 0)
	{
		char buf[128];
		int *aux;
		QString cmd;
		uchar temp[4];
		cmd = QString("D9:9*");
		if (sendCommand(cmd, buf, 10))
		{
			if (buf[0]=='D' and buf[1]=='9' and buf[2]==':' and buf[4]==':')
			{
				buf[9] = '\0';
				int autonomy = 30 * atoi(buf+5);
				bstate_mutex->lock();
				bstate_mutex->unlock();
			}
		}
	}

}

/**
* \brief Internal slot for computing base position from wheels position
*/
void RobexHandler::computePosBase()
{
	static QTime reloj=QTime::currentTime();
	int pl,pr;
	float dl,dr,incA=0.f,al=0.f;
	if ( getPosWheels ( pl,pr ) == true )
	{
		dl = float ( pl-lant );
		dr = float ( pr-rant );
		lant = pl;
		rant = pr;

		//Compute increments through robot kinematics
		//Avance en mm's
		incA = ( dl + dr ) *mechParams.temp ;
		//Rotación realizada en radianes
		al = atan2 ( ( dl - dr ) * 2. * mechParams.temp, mechParams.axisLength );
		//Update state struct
		float cos_al=cos(al);
		float sin_al=sin(al);
				
		bstate_mutex->lock();
			float cos_bs=cos(bState.alpha); 
			float sin_bs=sin(bState.alpha);
			float a =  cos_bs*cos_al + sin_bs*(-sin_al);
			float b =  cos_bs*sin_al + sin_bs*cos_al;
			float c = -sin_bs*cos_al + cos_bs*(-sin_al);
			float d = -sin_bs*sin_al + cos_bs*cos_al;
			bState.x += b * incA; 
			bState.z += d * incA;
			bState.alpha = -atan2(c,a);

			float Ccos_bs=cos(bState.correctedAlpha); 
			float Csin_bs=sin(bState.correctedAlpha);
			float Ca =  Ccos_bs*cos_al + Csin_bs*(-sin_al);
			float Cb =  Ccos_bs*sin_al + Csin_bs*cos_al;
			float Cc = -Csin_bs*cos_al + Ccos_bs*(-sin_al);
			float Cd = -Csin_bs*sin_al + Ccos_bs*cos_al;
			bState.correctedX += Cb * incA; 
			bState.correctedZ += Cd * incA;
			bState.correctedAlpha = -atan2(Cc,Ca);

			float t=(float)reloj.restart();
			//bState.x = incA;
			bState.alpha = al;
			bState.advVx = incA / t * 1000.f ;
			bState.rotV = al/ t * 1000.f;
			bState.isMoving = fabs(bState.x) > 0.5 or fabs(bState.alpha) > 0.005;
		bstate_mutex->unlock();	
	}
	else
		rError("Can't get wheels position");
}
/**
* \brief Internal slot for checking the motors timeout/state
*/
void RobexHandler::checkMotorsState()
{
	bstate_mutex->lock();
		bool aux = bState.isMoving;
	bstate_mutex->unlock();
	if ( aux==false && AreMotorsActivated && (LastCmdTimestamp.elapsed() > COMMAND_FREE_MOTORS_TIMEOUT )  ){
		if (  setMotorsState( MOTORS_STATE_FREE ) )
			rInfo("Motors state changed to free!");
		else
			rError("Warning, motors state could not be changed to 'free'" );
	}
}
/**
* \brief Set the break motor's state
* @param state New state ( MOTORS_STATE_NORMAL, MOTORS_STATE_FREE, MOTORS_STATE_BREAK )
* @return true if command was executed successfully, else return false
*/
bool RobexHandler::setMotorsState ( int state ){
	QString cmd;
	char buf[128]; //dummy
	cmd = MOTION_MOTORS_STATE_COMMAND_STRING;
	switch ( state ){
		case MOTORS_STATE_NORMAL:
			sendCommand("F0:0*", buf, 3);
			cmd += "00";
			break;
		case MOTORS_STATE_FREE:
			cmd += "22";
			break;
		case MOTORS_STATE_BREAK:
			cmd += "11";
			break;
		default:
			return false;
	}
	cmd += MOTION_MOTORS_STATE_COMMAND_CRC;
	bool success=sendCommand ( cmd , buf, 3 );
	//modifica la variable de estado de los motores
	if(success){
		if(state==0)
			AreMotorsActivated=true;
		else
			AreMotorsActivated=false;
	}
	return success;
}
/**
* \brief Set wheels speed in rads/sg
* @param left wheel speed 3 digits (unidades décimas centésimas sin coma)
* @param right wheel speed 3 digits (unidades décimas centésimas sin coma)
* @return true if command was executed successfully, else return false
*/
bool RobexHandler::setSpeedWheels ( float left, float right )
{
	QString cmd;
	bool status=false;
	char buf[128]; 

	if ( AreMotorsActivated == false )
		setMotorsState ( MOTORS_STATE_NORMAL );
	
	if ( AreMotorsActivated == false )
		rError("Warning, motors state could not be changed to 'normal operation'" );			
	else{
			cmd.resize ( 12 );
			cmd[0]='C';
			cmd[1]='1';
			cmd[2]=':';
			if ( left < 0 )
				cmd[3]='-';
			else
				cmd[3] = '+';
			int leftI = abs ( ( int ) rint ( left * 100 ) ) +1000;//1XXX
	
			QString Lvalue = QString::number ( leftI );
			cmd[4]=Lvalue[1];
			cmd[5]=Lvalue[2];
			cmd[6]=Lvalue[3];
			if ( right < 0 )
				cmd[7]='-';
			else cmd[7] = '+';
			int rightI = abs ( ( int ) rint ( right *100 ) ) +1000;//1XXX
			QString Rvalue = QString::number ( rightI );
	
			cmd[8]=Rvalue[1];
			cmd[9]=Rvalue[2];
			cmd[10]=Rvalue[3];
			cmd[11]='1';
			cmd[12]='*';
	
			LastCmdTimestamp.restart();
	
			status = sendCommand ( cmd , buf, 3 );
	}
	return status;
}

/**
* \brief Send command to the device
* @param cmd Command to send
* @return true if command was sent successfully, else return false
*/
bool RobexHandler::sendCommand(QString cmd, char *buf, int totalread){
	char *command;
	if(MotionDevice.isOpen()){
		cmd += 'X';
		cmd += 'X';
		command = cmd.toLatin1().data();
	
		for (int o = 0; o < cmd.size(); ++o) stringEnviado[o] = (uchar)(cmd[o].toLatin1());// 	stringEnviado = cmd;
	
		// Send command
		MotionDevice.write(command,cmd.size());
		numRecibido = MotionDevice.read(buf, totalread);
		
		
		for (int i = 0; i < numRecibido; ++i) 
			stringRecibido[i] = (unsigned char)buf[i];
	
		if ( numRecibido != totalread ){
		       
			printf("[10] baseComp: Ack error: <comand(%d) ", cmd.size());
			return false;
		}
		else{
			for (int o=0; o<numRecibido; ++o) stringRecibido[o] = buf[o]; // stringRecibido = buf; /// XXX
			return true;
		}
	}
	else{
		rError("Fatal error, motion deviced closed");
		return false;
	}
}
//Utilities
/**
* \brief Utility function. Transform ascii to decimal
* @param buf Ascci string
* @param ndigits Number of digits
* @param out Returned integer
* @return true if transform done successfully
*/
bool RobexHandler::ascii2dec ( char *buf, int ndigits, int &out ){
	int i;
	out = 0;
	for ( i=0; i<ndigits; i++ )
	{
		if ( buf[i] < 48 || buf[i] > 57 ) return false;
		out += ( buf[i] - 48 ) * ( int ) pow ( 10,ndigits-1-i );
	}
	return true;
}

void RobexHandler::correctOdometer(float x, float z, float alpha)
{
	bState.correctedX += x;
	bState.correctedZ += z;
	bState.correctedAlpha += alpha;
}

