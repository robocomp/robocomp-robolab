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
#include "dynamixel_serial.h"

Dynamixel::Dynamixel(  RoboCompJointMotor::BusParams  *busParams , RoboCompJointMotor::MotorParamsList *params, QMutex *m)
{
	h_mutex = new QMutex();
	m_mutex = new QMutex();
	this->busParams = busParams;
	this->params = params;
	w_mutex = m;
}

Dynamixel::~Dynamixel()
{
}

void Dynamixel::initialize() throw (QString)
{
	QString device = QString::fromStdString( busParams->device);
	qDebug()<<"\n\n||  DYNAMIXEL::initialize -----> DEVICE: "<<device<<"   ||";

	// 1) Identifiers of the FTDI (in order to identifie the usb with the dynamixel):
	//---> 	ID_VENDOR  = 0x0403
	//---> 	ID_PRODUCT = 0x6001
	int counter = 0;
	struct usb_bus    *bus;
	struct usb_device *dev;
	usb_init();
	usb_find_busses();  //get the busses
	usb_find_devices(); //get the devices
	for (bus = usb_busses; bus; bus = bus->next)
	{
		for (dev = bus->devices; dev; dev = dev->next)
		{
			if(dev->descriptor.idVendor == 0x0403 and dev->descriptor.idProduct == 0x6001) counter++;
		}
	}
/*	if (counter == 2)  qDebug()<<"||  DYNAMIXEL::initialize -----> USB CONNECTED!!  ||";
	else
	{
		if (counter > 0) counter--;
		qDebug()<<"||  ERROR DYNAMIXEL serial::initialize ---> The USB is not connected. Dynamixel locate "<<counter<<" times  ||";
		qFatal("Aborted");
	}*/

	// 2) Open and initialize the device
	port.setName( device );
	if (port.open(device) == false)
	{
		//The Dynamixel device failed because:
		// 1) You don't have the necessary permissions
		// 2) The USB is not connected
		QString error;
		QFile::Permissions p = QFile::permissions(QString::fromStdString(busParams->device)); //contains FLAGS
		//Sacamos el flag de permiso de escritura de propietario. Si no es verdadero (0x2000) entonces no tenemos
		//permiso para ejecutar el dynamixel en el puerto
		if ( (p | QFile::WriteOwner) != true)
			error = "||  ERROR DYNAMIXEL::initialize ---> Port " + QString::fromStdString(busParams->device) +
					" could not be opened. You don't have write permission on the device. Try 'sudo chmod 777 " +
					QString::fromStdString( busParams->device ) + "'   ||";
		else
			error = "||  ERROR DYNAMIXEL::initialize ---> Port " + QString::fromStdString( busParams->device ) +
					" could not be opened. Please check file permissions   ||";
		throw error;
	}

	// 3) Setting baudrate
	QSerialPort::_BaudRateType bRate;
	switch ( busParams->baudRate )
	{
		case 2400:    bRate = QSerialPort::BAUD2400 ;   break;
		case 4800:    bRate = QSerialPort::BAUD4800 ;   break;
		case 9600:    bRate = QSerialPort::BAUD9600 ;   break;
		case 19200:   bRate = QSerialPort::BAUD19200 ;  break;
		case 38400:   bRate = QSerialPort::BAUD38400 ;  break;
		case 57600:   bRate = QSerialPort::BAUD57600 ;  break;
		case 76800:   bRate = QSerialPort::BAUD76800 ;  break;
		case 115200:  bRate = QSerialPort::BAUD115200 ; break;
		case 230400:  bRate = QSerialPort::BAUD230400 ; break;
		default:      bRate = QSerialPort::BAUD115200 ; break;
	}
	port.setBaudRate( bRate ); //set the baudrate.
	if(port.baudRate() != bRate ) qFatal("||  ERROR DYNAMIXEL::initialize!! ---> Error setting Baud Rate %d\n   ||", busParams->baudRate);
	qDebug()<<"||  DYNAMIXEL::initialize -----> baudRate: "<<bRate<<", Valor"<<busParams->baudRate<<"   ||";

	//4) Create servos instances in a QMap indexed by name
	for (int i = 0; i < busParams->numMotors; i++)
	{
		std::cout << "JointMotor::Dynamixel::Dynamixel - name " <<params->operator[](i).name<< std::endl;
		QString name = QString::fromStdString(params->operator[](i).name);
		motors[name] = new Servo( params->operator[](i) );
		//steps_range , max_degrees, steps_speed_range, max_speed_rads  ¿¿Deberían venir en los parámetros?
		// Fixed the max velocity motor to 12 rad/s to get the desired velocity from the config file (maxSpeed in Rad/s)
		motors[name]->setMotorRanges(params->operator[](i).stepsRange,params->operator[](i).maxDegrees, 1024, 12);
	}

	std::cout << "JointMotor::Dynamixel::Dynamixel - Motor Map created with " << busParams->numMotors << " motors: " << std::endl;
	foreach( Servo * s, motors.values())
	{
		std::cout << "	" + s->params.name << std::endl;
	}

	//Initialize class variables
	bzero(packet, MAX_LENGTH_PACKET); //The bzero() function sets the first n bytes of the area starting at s to zero (bytes containing '\0').
	packet[0]=0xFF;
	packet[1]=0xFF;
	//Initialize motor params
	foreach( Servo *servo, motors.values() )
	{
		Servo::TMotorData               &data   = servo->data;
		RoboCompJointMotor::MotorParams &params = servo->params;

		std::cout << "JointMotor::Dynamixel::Dynamixel - Configuration data of motor " << params.name << std::endl;
		///Set Status return level to 1.
		// --> Level 0: no response ;
		// --> Level 1: only for reading commands. Default ;
		// --> Level 2: always
		int level = 1;
		setStatusReturnLevel(params.busId, level);
		getStatusReturnLevel(params.busId, level);

		//set specific parameters
	    setPunch(params.busId, 32);
		setBothComplianceMargins(params.busId, 1);
		setBothComplianceSlopes(params.busId, 64);

		bool usbCorrect = true;
		///Return delay time
		int rt = 50;
		if (setReturnDelayTime( params.busId, rt) == true and getReturnDelayTime( params.busId, rt) == true)
		{
			qDebug() << "	Return delay time: " << rt;
		}
		else
		{
			qDebug() << "Error setting delay time";
			usbCorrect = false;
		}
		///Control params
		int m;
		if (getPunch( params.busId, m ) == true)
		{
			qDebug() << "	Punch: " << m;
		}
		else
		{
			qDebug() << "Error reading Punch";
			usbCorrect = false;
		}

		if (getCCWComplianceMargin( params.busId, m ) == true)
		{
			qDebug() << "	CCWComplianceMargin: " << m;
		}
		else
		{
			qDebug() << "Error reading CCWComplianceMargin";
			usbCorrect = false;
		}

		if (getCWComplianceMargin( params.busId, m ) == true)
		{
			qDebug() << "	CWComplianceMargin: " << m;
		}
		else
		{
			qDebug() << "Error reading CWComplianceMargin";
			usbCorrect = false;
		}
		if (getCCWComplianceSlope( params.busId, m ) == true)
		{
			qDebug() << "	CCWComplianceSlope: " << m;
		}
		else
		{
			qDebug() << "Error reading CCWComplianceSlope";
			usbCorrect = false;
		}

		if (getCWComplianceSlope( params.busId, m ) == true)
		{
			qDebug() << "	CWComplianceSlope: " << m;
		}
		else
		{
			qDebug() << "Error reading CWComplianceSlope";
			usbCorrect = false;
		}
		
		if (!usbCorrect)
		{
			qFatal("Can't initialize dynamyxel");
		}

		///Read current position
		float p;
		getPosition(QString::fromStdString(params.name), p );
		data.currentPosRads  = p;
		data.antPosRads = p;
		data.currentPos = servo->rads2Steps(p);
		qDebug() << "	Current position (steps): " << data.currentPos;
		qDebug() << "	Current position (rads): " << data.currentPosRads;

		///Set limits
//		qDebug()<<"inverted"<<params.invertedSign;
		if(params.invertedSign == true)
		{
			if( setMaxPosition(QString::fromStdString(params.name), params.minPos) != true)
				qDebug()<<" Error setting min Position "<<params.minPos; 
			if( setMinPosition(QString::fromStdString(params.name), params.maxPos) != true)
				qDebug()<<" Error setting max Position "<<params.maxPos;
		}
		else
		{
			if( setMaxPosition(QString::fromStdString(params.name), params.maxPos)!= true)
				qDebug()<<" Error setting min Position "<<params.minPos; 
			if( setMinPosition(QString::fromStdString(params.name), params.minPos) != true)
				qDebug()<<" Error setting max Position "<<params.maxPos;
		}
		
		getMaxPosition( QString::fromStdString(params.name), p);
		qDebug() << "	Max position (nominal/steps/rads): " << params.maxPos << servo->steps2Rads(p);
		getMinPosition( QString::fromStdString(params.name), p);
		qDebug() << "	Min position (nominal/steps/rads): " << params.minPos << servo->steps2Rads(p);

		///set servos to maximum speed
		data.maxVelocityRads = params.maxVelocity;
		setVelocity( QString::fromStdString(params.name), params.maxVelocity);

		qDebug() << "	Max velocity " << params.maxVelocity;
	}
	
}

void Dynamixel::update() throw(MotorHandlerUnknownMotorException,MotorHandlerErrorWritingToPortException)
{
  //qDebug() << "Entering update " ;
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
			//throw ex;
		}
		catch(const MotorHandlerErrorWritingToPortException &ex)
		{
			//throw ex;
		}
	}
}

///////////Private methods/////////////

bool Dynamixel::sendIPacket( char *p, char length )
{
	if ( port.write( p , length) == length )
	  return true;
	else
	  return false;
}

/// 0xFF 0xFF
///	ID
///	LENGTH : params +2
///	ERROR :
/*					Bit        Name                                      Details
					Bit 7         0                                          -
											Set to 1 if an undefined instruction is given without the
					Bit 6 Instruction Error
											reg_write instruction.
					Bit 5  Overload Error Set to 1 if the specified torque can't control the load.
											Set to 1 if the checksum of the intruction packet is
					Bit 4 Checksum Error
											incorrect
					Bit 3    Range Error    Set to 1 if the instruction is out of the usage range.
							Overheating    Set as 1 if the internal temperature of Dynamixel is out of
					Bit 2
								Error       the operative range as set in the control table.
											Set as 1 if the goal position is set outside of the range
					Bit 1 Angle Limit Error
											between CW Angle Limit and CCW Angle Limit
							Input Voltage   Set to 1 if the voltage is out of the operative range set in
					Bit 0r)
								Error       the control table.*/
///	PARAMS0--N
///	CHECKSUM
/*					Check Sum = ~( ID + Length + Instruction + Parameter1 + … Parameter N )
					If the calculated value is bigger than 255, the lower byte becomes the checksum.
					~ represents the Not or complement operation*/

bool Dynamixel::getSPacket( )
{
	if ( port.read(status,5) != 5)
	  return false;

	int pars = status[3]-2;
	port.read(status+5,pars + 1);

 	char cs = checkSum( status );
 	if( cs != status[5+pars] )
	{
		qDebug() << "Checksum error reading status packet";
		return false;
	}
	else return true;
}

void Dynamixel::printPacket(char *packet)
{
	int pars = packet[3]-2;

 	printf( "PrintPacket: %d %d %d %d %d ",(uchar)packet[0],(uchar)packet[1],(uchar)packet[2],(uchar)packet[3],(uchar)packet[4]);
	for(int i=5; i< 5+pars;i++)
	{
		printf(" %d ",(uchar)packet[i]);
	}
	printf(" %d \n", (uchar)packet[5+pars]);
}

char Dynamixel::checkSum(char *packet)
{
	int sum = 0;
	sum = packet[2]+packet[3]+packet[4];
	for(int i=5; i < 5+packet[3]-2; i++)
		sum += packet[i];
	return (char)~sum;
}

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


//////////Abstract class implementation//////////////////

/******************************************************************************/
/* Send a position read command to AI-motor                                   */
/* Input : ServoID                                                            */
/* Output : Position                                                          */
/******************************************************************************/
void Dynamixel::getPosition(const QString &motor, float  &pos) throw(MotorHandlerUnknownMotorException,MotorHandlerErrorWritingToPortException)
{
	QMutexLocker locker(h_mutex);
	packet[2]=motors[motor]->params.busId;
	packet[3]=0x04;
	packet[4]=READ_DATA;
	packet[5]=0x24;
	packet[6]=0x02;
	packet[7]=checkSum(packet);

	if ( sendIPacket(packet, 8) == false )
	  throw MotorHandlerErrorWritingToPortException("Dynamixel::getPosition");
	if ( getSPacket() == false )
	  throw MotorHandlerErrorWritingToPortException("Dynamixel::getPosition");
	else
	{
	  int posInt = *((unsigned short *)(status+5));
	  pos = motors[motor]->steps2Rads( posInt );
	}
}

void Dynamixel::setPosition( const QString &motor,  float  pos, float maxSpeed ) throw(MotorHandlerUnknownMotorException,MotorHandlerErrorWritingToPortException)
{
	QMutexLocker locker(h_mutex);

	if( motors.contains( motor ) )
	{
	  Servo *servo = this->motors[motor];
// 	  if ( servo->data.currentVelocityRads != maxSpeed )
// 		setVelocity(motor, servo->data.maxVelocityRads);
	  
		if(maxSpeed < -servo->data.maxVelocityRads)
			setVelocity(motor,-servo->data.maxVelocityRads);
		else
			if(maxSpeed > servo->data.maxVelocityRads)
				setVelocity(motor,servo->data.maxVelocityRads);
			else
				setVelocity(motor, maxSpeed);
			
	
	  int pInt = this->motors[motor]->rads2Steps(pos);
//	  qDebug()<<"setting position"<<pInt<<"of motor"<<motor<<motors[motor]->params.busId<<pos;
	  packet[2]=motors[motor]->params.busId;
	  packet[3]=0x05;
	  packet[4]=WRITE_DATA;
	  packet[5]=0x1E;
	  packet[6]=pInt;
	  packet[7]=pInt>>8;
	  packet[8]=checkSum(packet);

	  if ( sendIPacket(packet, 9) == false )
	  {
		throw MotorHandlerErrorWritingToPortException("Dynamixel::setPosition");
		qDebug()<<"sendIpacket";
		
	  }

	}
	else
	{
	  qDebug()<<"motor.contains";
	  throw MotorHandlerUnknownMotorException( motor.toStdString());
	}
//	qDebug()<<"done:setting position of motor"<<motor;

}

void Dynamixel::setSyncPosition( const QVector<Dynamixel::GoalPosition> & goals) throw( MotorHandlerErrorWritingToPortException )
{
	QMutexLocker locker(h_mutex);
	int numMotors = goals.size();
	//Lenght = (L+1)xN+4 L:data length per motor; N number of motors
	uchar size = (4 + 1) * numMotors + 4;
	int i,k,p,v;

	packet[2]=BROADCAST;
	packet[3]=size;	//Length
	packet[4]=SYNC_WRITE;
	packet[5]=0x1E;	//Start address to write data
	packet[6]=0x04;	//Length of data to write
	for(i = 7, k=0; i < 7 + (5 * numMotors); i = i + 5, k++)
	{
	  packet[i] = goals[k].busDir;	// Address of first motor
	  p = motors[goals[k].name]->rads2Steps(goals[k].position);
	  packet[i+1] = p;
	  packet[i+2] = p>>8;
	  v = motors[goals[k].name]->radsPerSec2Steps(goals[k].maxVel);
	  packet[i+3] = v;
	  packet[i+4] = v>>8;
	}
	packet[i]=checkSum(packet);

	if ( sendIPacket(packet, i+1) == false )
	{
	  throw MotorHandlerErrorWritingToPortException("Dynamixel::setSyncPosition");
	}

}


bool Dynamixel::getVelocity(const QString &motor, float  & vel )
{
	QMutexLocker locker(h_mutex);

	packet[2]=motors[motor]->data.busId;
	packet[3]=0x04;
	packet[4]=READ_DATA;
	packet[5]=0x26;
	packet[6]=0x02;
	packet[7]=checkSum(packet);

	if ( sendIPacket(packet, 8) == true and getSPacket() == true )
	{
	  vel = *((unsigned short *)(status+5));
	  return true;
	}
	else
	  return false;
}

bool Dynamixel::setVelocity( const QString &motor, float  vel )
{
// 	QMutexLocker locker(h_mutex);

	packet[2]=motors[motor]->data.busId;
	packet[3]=0x05;
	packet[4]=WRITE_DATA;
	packet[5]=0x20;
	int v = motors[motor]->radsPerSec2Steps(vel);
	packet[6]=v;
	packet[7]=v>>8;
	packet[8]=checkSum(packet);


	if ( sendIPacket(packet, 9) == false)
	  return false;

//	qDebug()<<"velocity"<<v<<"of motor"<<motor<<motors[motor]->params.busId;
//	printPacket(packet);

	motors[motor]->data.currentVelocityRads = vel;
	return true;
}

bool Dynamixel::getMaxPosition( const QString &motor, float  & pos )
{
	QMutexLocker locker(h_mutex);

	packet[2]=motors[motor]->data.busId;
	packet[3]=0x04;
	packet[4]=READ_DATA;
	packet[5]=0x08;
	packet[6]=0x02;
	packet[7]=checkSum(packet);

	if ( sendIPacket(packet, 8) == true and getSPacket() == true )
	{
	  pos = *((unsigned short *)(status+5));
	  return true;
	}
	return false;
}

bool Dynamixel::getMinPosition( const QString &motor, float  & pos )
{
	QMutexLocker locker(h_mutex);

	packet[2]=motors[motor]->data.busId;
	packet[3]=0x04;
	packet[4]=READ_DATA;
	packet[5]=0x06;
	packet[6]=0x02;
	packet[7]=checkSum(packet);

	if ( sendIPacket(packet, 8) == true and getSPacket() == true )
	{
	  pos = *((unsigned short *)(status+5));
	  return true;
	}
	return false;
}

bool Dynamixel::setMaxPosition( const QString &motor, float  pos )
{
	QMutexLocker locker(h_mutex);

	packet[2]=motors[motor]->data.busId;
	packet[3]=0x05;
	packet[4]=WRITE_DATA;
	packet[5]=0x08;
	int p = motors[motor]->rads2Steps(pos);
	packet[6]=p;
	packet[7]=p>>8;
	packet[8]=checkSum(packet);

	if (sendIPacket(packet, 9) == false)
	  return false;

	return true;
}

bool Dynamixel::setMinPosition( const QString &motor, float  pos )
{
	QMutexLocker locker(h_mutex);

	packet[2]=motors[motor]->data.busId;
	packet[3]=0x05;
	packet[4]=WRITE_DATA;
	packet[5]=0x06;
	int p = motors[motor]->rads2Steps(pos);
	packet[6]=p;
	packet[7]=p>>8;
	packet[8]=checkSum(packet);

	if (sendIPacket(packet, 9) == false)
	  return false;

	return true;
}

bool Dynamixel::powerOn( const QString &motor )
{
	QMutexLocker locker(h_mutex);

	packet[2]=motors[motor]->data.busId;
	packet[3]=0x05;
	packet[4]=WRITE_DATA;
	packet[5]=0x18;
	packet[6]=0x01;
	packet[7]=0x01;
	packet[8]=checkSum(packet);

	if ( sendIPacket(packet, 9) == false)
	  return false;

	return true;
}

bool Dynamixel::powerOff( const QString &motor )
{
	QMutexLocker locker(h_mutex);

	packet[2]=motors[motor]->data.busId;
	packet[3]=0x05;
	packet[4]=WRITE_DATA;
	packet[5]=0x18;
	packet[6]=0x00;
	packet[7]=0x00;
	packet[8]=checkSum(packet);

	if ( sendIPacket(packet, 9) == false)
	  return false;

	return true;
}

bool Dynamixel::getPower( const QString &motor, float & pow )
{
	QMutexLocker locker(h_mutex);

	packet[2]=motors[motor]->data.busId;
	packet[3]=0x04;
	packet[4]=READ_DATA;
	packet[5]=0x28;
	packet[6]=0x02;
	packet[7]=checkSum(packet);

	if ( sendIPacket(packet, 8) == true and getSPacket() == true )
	{
	  char aux = status[6];
	  status[6] = status[6] & 0x04;
	  pow = *((unsigned short *)(status+5));
	  if((aux >> 2) == 1)
		pow = -pow;
	  return true;
	}
	return false;
}

bool Dynamixel::setId( const QString &motor, int id )
{
	QMutexLocker locker(h_mutex);

	packet[2]=motors[motor]->data.busId;
	packet[3]=0x04;
	packet[4]=WRITE_DATA;
	packet[5]=0x03;
	packet[6]=id;
	packet[7]=checkSum(packet);

	if (sendIPacket(packet, 8) == false)
	  return false;

	return false;
}

bool Dynamixel::setBaudrate( const QString &motor, int baud )
{
	QMutexLocker locker(h_mutex);

	packet[2]=motors[motor]->data.busId;
	packet[3]=0x04;
	packet[4]=WRITE_DATA;
	packet[5]=0x04;
	packet[6]=0x01;
	packet[7]=checkSum(packet);

	if( sendIPacket(packet, 8) == false)
	  return false;

	return true;
}

bool Dynamixel::getBaudrate(uchar motor, int &br)
{
	QMutexLocker locker(h_mutex);

	packet[2]=motor;
	packet[3]=0x04;
	packet[4]=READ_DATA;
	packet[5]=0x04;
	packet[6]=0x01;
	packet[7]=checkSum(packet);

	if ( sendIPacket(packet, 8) == true and getSPacket() == true )
	{
	  br = (int)status[5];
	  return(true);
	}
	return false;
}

/**
 * Sets Voltage Limit
 * @param motor
 * @param limit
 * @return
 */
bool Dynamixel::setVoltageLimit( uchar motor, int limit )
{
	QMutexLocker locker(h_mutex);

	packet[2]=motor;
	packet[3]=0x04;
	packet[4]=WRITE_DATA;
	packet[5]=0x0C;
	packet[6]=limit;
	packet[7]=limit<<8;
	packet[8]=checkSum(packet);

	if( sendIPacket(packet, 8) == false)
	  return false;

	return true;
}

bool Dynamixel::getVoltageLimit( uchar motor, int & limit )
{
	QMutexLocker locker(h_mutex);

	packet[2]=motor;
	packet[3]=0x04;
	packet[4]=READ_DATA;
	packet[5]=0x0C;
	packet[6]=0x02;
	packet[7]=checkSum(packet);

	if ( sendIPacket(packet, 8) == true and getSPacket() == true )
	{
	  limit = status[5];
	  return true;
	}
	return false;
}

bool Dynamixel::isMoving( const QString &motor )
{
	QMutexLocker locker(h_mutex);

	packet[2]=motors[motor]->data.busId;
	packet[3]=0x04;
	packet[4]=READ_DATA;
	packet[5]=0x2E;
	packet[6]=0x01;
	packet[7]=checkSum(packet);

	if ( sendIPacket(packet, 8) == true and getSPacket() == false )  //AMBIGUO
	  return false;
 	if (status[5]==0)
	  return false;
 	else return true;
}

bool Dynamixel::getPunch(uchar motor, int & d) //Named PUNCH in dynamixel manual
{
	QMutexLocker locker(h_mutex);

	packet[2]=motor;
	packet[3]=0x04;
	packet[4]=READ_DATA;
	packet[5]=0x30;
	packet[6]=0x02;
	packet[7]=checkSum(packet);

	if (sendIPacket(packet, 8) == true and getSPacket() == true	 )
	{
	  d = *((unsigned short *)(status+5));
	  return true;
	}
	return false;
}

bool Dynamixel::setPunch(uchar motor, int d)
{
	QMutexLocker locker(h_mutex);
	if ( d<32 or d> 1023 )
	{
		qWarning("Punch parameter out of range:%d. Try with a 0:16000 value",d);
		return false;
	}

	packet[2]=motor;
	packet[3]=0x05;
	packet[4]=WRITE_DATA;
	packet[5]=0x30;
	packet[6]=d;
	packet[7]=d>>8;
	packet[8]=checkSum(packet);

	if( sendIPacket(packet, 9) == false)
	  return false;

	return true;
}

bool Dynamixel::setBothComplianceSlopes(uchar motor, int p) //Compliance slope in both rotation senses
{
	QMutexLocker locker(h_mutex);

	if ( p<0 or p> 255 )
	{
		qWarning("Compliance parameter out of range:%d. Try with a 0:255 value",p);
		return false;
	}
	packet[2]=motor;
	packet[3]=0x04;
	packet[4]=WRITE_DATA;
	packet[5]=0x1C;
	packet[6]=(uchar)p;
	packet[7]=checkSum(packet);
	sendIPacket(packet, 8);

	packet[2]=motor;
	packet[3]=0x04;
	packet[4]=WRITE_DATA;
	packet[5]=0x1D;
	packet[6]=(uchar)p;
	packet[7]=checkSum(packet);

	if( sendIPacket(packet, 8) == false )
	  return false;

	return true;
}


bool Dynamixel::setBothComplianceMargins(uchar motor, int db)
{
	QMutexLocker locker(h_mutex);

	packet[2]=motor;
	packet[3]=0x04;
	packet[4]=WRITE_DATA;
	packet[5]=0x1A;
	packet[6]=(uchar) db;
	packet[7]=checkSum(packet);

	if( sendIPacket(packet, 8) == false)
	  return false;

	packet[2]=motor;
	packet[3]=0x04;
	packet[4]=WRITE_DATA;
	packet[5]=0x1B;
	packet[6]=(uchar) db;
	packet[7]=checkSum(packet);

	if( sendIPacket(packet, 8) == false)
	  return false;

	return true;
}

bool Dynamixel::getCWComplianceMargin(uchar motor, int & db)
{
	QMutexLocker locker(h_mutex);

	packet[2]=motor;
	packet[3]=0x04;
	packet[4]=READ_DATA;
	packet[5]=0x1A;
	packet[6]=0x01;
	packet[7]=checkSum(packet);

	if( sendIPacket(packet, 8) == true and getSPacket() == true )
	{
	  db = (int)(status[5]);
	  return true;
	}
	return false;
}

bool Dynamixel::getCCWComplianceMargin(uchar motor, int & db)
{
	QMutexLocker locker(h_mutex);

	packet[2]=motor;
	packet[3]=0x04;
	packet[4]=READ_DATA;
	packet[5]=0x1B;
	packet[6]=0x01;
	packet[7]=checkSum(packet);

	if( sendIPacket(packet, 8) == true and getSPacket() == true )
	{
	  db = (int)(status[5]);
	  return true;
	}
	return false;
}

bool Dynamixel::getCWComplianceSlope(uchar motor, int & db)
{
	QMutexLocker locker(h_mutex);

	packet[2]=motor;
	packet[3]=0x04;
	packet[4]=READ_DATA;
	packet[5]=0x1C;
	packet[6]=0x01;
	packet[7]=checkSum(packet);

	if( sendIPacket(packet, 8) == true and getSPacket() == true )
	{
	  db = (int)(status[5]);
	  return true;
	}
	return false;
}

bool Dynamixel::getCCWComplianceSlope(uchar motor, int & db)
{
	QMutexLocker locker(h_mutex);

	packet[2]=motor;
	packet[3]=0x04;
	packet[4]=READ_DATA;
	packet[5]=0x1D;
	packet[6]=0x01;
	packet[7]=checkSum(packet);

	if( sendIPacket(packet, 8) == true and getSPacket() == true )
	{
	  db = (int)(status[5]);
	  return true;
	}
	return false;
}

bool Dynamixel::setReturnDelayTime( uchar motor, int t)
{
	QMutexLocker locker(h_mutex);

	packet[2]=motor;
	packet[3]=0x04;
	packet[4]=WRITE_DATA;
	packet[5]=0x05;
	packet[6]=t;
	packet[7]=checkSum(packet);

	if ( sendIPacket(packet, 8) == false)
	  return false;

	return true;
}

bool Dynamixel::getReturnDelayTime( uchar motor, int & t)
{
  QMutexLocker locker(h_mutex);

  packet[2]=motor;
  packet[3]=0x04;
  packet[4]=READ_DATA;
  packet[5]=0x05;
  packet[6]=0x01;
  packet[7]=checkSum(packet);

  if (sendIPacket(packet, 8) == true and getSPacket() == true )
  {
	t = (int)status[5];
	return true;
  }

  return false;
}
/**
 * \brief GET STATUS RETURN LEVEL
 * @param motor bus identification
 * @param level the level of the motor status
 */ 
bool Dynamixel::getStatusReturnLevel(uchar motor,  int &level)
{
	packet[2] = motor;
	packet[3] = 0x04;
	packet[4] = READ_DATA;
	packet[5] = 0x10;
	packet[6] = 0x01;  //level??
	packet[7] = checkSum(packet);

	if(sendIPacket(packet, 8) == true and getSPacket() == true)
	{
		level = status[5];
		return true;
	}
	else
		return false;
}
/**
 * \brief SET STATUS RETURN LEVEL
 * @param motor bus identification
 * @param level the level of the motor status
 */ 
bool Dynamixel::setStatusReturnLevel(uchar motor,  int &level)
{
	packet[2] = motor;
	packet[3] = 0x04;
	packet[4] = WRITE_DATA;
	packet[5] = 0x10;
	packet[6] = level;
	packet[7] = checkSum(packet);

	if(sendIPacket(packet, 8) == true) 
		return true;
	else                              
		return false;
}
bool Dynamixel::getTemperature(const QString &motor,int &temperature)
{
	packet[2] = motors[motor]->data.busId;
	packet[3]=0x04;
	packet[4]=READ_DATA;
	packet[5]=0x2B;
	packet[6]=0x01;
	packet[7]=checkSum(packet);
	if(sendIPacket(packet,8) == true and getSPacket() == true)
	{
		temperature = status[5];
		return true;
	}
	else
		return false;
}
