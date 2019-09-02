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
#ifndef DYNAMIXEL_SERIAL_H
#define DYNAMIXEL_SERIAL_H

#include <QObject>
#include <QtCore>

#include <iostream>

#include <q4serialport/q4serialport.h>
// sudo apt-get install libusb-dev
#include <usb.h>

#include "TSQHash.h"
#include "JointMotor.h"
#include "handler.h"
#include "servo.h"

#define BROADCAST 0xFE
#define SYNC_WRITE 0x83
#define WRITE_DATA 0x03
#define READ_DATA 0x02
#define ACTION 0x05
#define REG_WRTIE 0x04
#define PING 0x01
#define RESET 0x06
#define MAX_LENGTH_PACKET 30

#define MINSTEPSPOSITION 0
#define MAXSTEPSPOSITION 1023
//
#define NO_RESPONSE
#define READ_RESPONSE
#define ALL_RESPONSE


/**
	*	Abstract class for servo motors handlers that allow bus access to several motors
	@author Pablo Bustos - RoboLab - UEx
*/
class Dynamixel : public Handler
{
public:
	Dynamixel(RoboCompJointMotor::BusParams  *busParams, RoboCompJointMotor::MotorParamsList *params, QMutex *m);
	~Dynamixel();

	virtual void initialize() throw(QString);
	RoboCompJointMotor::MotorParamsList getMotorParams( uchar motor){ return this->params[motor];};
	virtual void setPosition(const QString &motor,  float  pos, float maxSpeed) throw(MotorHandlerUnknownMotorException,MotorHandlerErrorWritingToPortException);
	virtual void setSyncPosition( const QVector<Dynamixel::GoalPosition> & goals) throw(MotorHandlerErrorWritingToPortException);
	virtual void getPosition(const QString &motor, float  &pos) throw(MotorHandlerUnknownMotorException,MotorHandlerErrorWritingToPortException);
	virtual bool getVelocity(const QString &motor, float  &vel);
	virtual bool setVelocity(const QString &motor, float  vel);
	virtual bool getReferenceVelocity(uchar  motor, float &vel){return false;};
	virtual bool setReferenceVelocity(uchar  motor, float vel){return false;};
	virtual bool getPower(const QString &motor,float&pow);
	virtual bool getMaxPosition(const QString &motor, float  &pos);
	virtual bool getMinPosition(const QString &motor, float  &pos);
	virtual bool setMaxPosition(const QString &motor, float  pos);
	virtual bool setMinPosition(const QString &motor, float  pos);
	virtual bool isMoving(const QString &motor);
	virtual bool getPGain(uchar motor, float &p){motor=motor;p=p;return false;};
	virtual bool getDGain(uchar motor,float&d){motor=motor;d=d;return false;};
	virtual bool getIGain(uchar motor,float&i){motor=motor;i=i;return false;};
	virtual bool setPGain(uchar motor,float p){motor=motor;p=p;return false;};
	virtual bool setDGain(uchar motor,float d){motor=motor;d=d;return false;};
	virtual bool setIGain(uchar motor,float i){motor=motor;i=i;return false;};
	virtual bool disablePWM(uchar motor){return false;};
	virtual bool enablePWM(uchar motor){return false;};
	virtual bool setId(const QString &motor,int id);
	virtual bool reset(uchar motor){motor=motor;return false;};
	virtual bool restoreDefValues(uchar motor){motor=motor;return false;};
	virtual bool stop(uchar motor){motor=motor;return false;};
	virtual bool powerOff(const QString &motor);
	virtual bool powerOn(const QString &motor);
	virtual bool setBaudrate(const QString &motor, int baud);
	virtual bool getBaudrate(uchar motor, int& baud);
	virtual void update() throw(MotorHandlerUnknownMotorException,MotorHandlerErrorWritingToPortException);

	//Out of abstract Handler. Specific of Dynamixel
	bool getCWComplianceMargin( uchar motor, int & m);
	bool getCCWComplianceMargin( uchar motor, int & m);
	bool setBothComplianceMargins( uchar motor, int m);
	bool getCWComplianceSlope( uchar motor, int & m);
	bool getCCWComplianceSlope( uchar motor, int & m);
	bool setBothComplianceSlopes( uchar motor, int  m);
	bool setReturnDelayTime( uchar motor, int t);
	bool getReturnDelayTime( uchar motor, int & t);
	bool getPunch( uchar motor, int & p);
	bool setPunch( uchar motor, int p);
	bool getVoltageLimit( uchar motor, int & v);
	bool setVoltageLimit( uchar motor, int v);
	bool getStatusReturnLevel(uchar motor,  int &level);
	bool setStatusReturnLevel(uchar motor,  int &level);
	bool getTemperature(const QString &motor,int &temperature);


	QMutex *h_mutex;
	QMutex *w_mutex;
	QMutex *m_mutex;

private:
	bool sendIPacket(char *packet, char length);
	bool getSPacket( );
	QSerialPort  port;
	char checkSum(char *packet);
	// STATUS structure:
	// status[0]
	// status[1]
	// status[2] motor name (bus Id)
	// status[3]
	// status[4] PING
	// status[5]
	char status[MAX_LENGTH_PACKET]; 
	void printPacket(char *packet);
	void ping(char motor);

	char packet[MAX_LENGTH_PACKET]; // vector char de 30 elementos??
	RoboCompJointMotor::BusParams *busParams;
	RoboCompJointMotor::MotorParamsList *params;

};

#endif

