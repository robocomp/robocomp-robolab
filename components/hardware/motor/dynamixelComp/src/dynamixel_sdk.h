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

#ifndef DYNAMIXEL_SDK_H
#define DYNAMIXEL_SDK_H

#include <QObject>
#include <QtCore>
#include <iostream>

#include <qlog/qlog.h>
#include "handler.h"
#include "JointMotor.h"
#include "TSQHash.h"
#include "servo.h"

#include <dynamixel.h>
#define ID_REGISTER					3
#define BAUD_RATE					4
#define RETURN_DELAY_TIME			5
#define MIN_POSITION				6
#define MAX_POSITION				8
#define VOLTAGE_LIMIT				12
#define STATUS_RETURN_VALUE 		16
#define SET_POWER	 				24
#define SET_CW_COMPLIANCE_MARGIN 	26
#define SET_CCW_COMPLIANCE_MARGIN	27
#define SET_CW_COMPLIANCE_SLOPE		28
#define SET_CCW_COMPLIANCE_SLOPE	29
#define SET_GOAL_POSITION_L 		30
#define SET_VELOCITY				32
#define GET_PRESENT_POSITION_L 		36
#define GET_VELOCITY				38
#define PRESENT_LOAD				40
#define GET_TEMPERATURE				43
#define MOVING						46
#define PUNCH						48

#define MINSTEPSPOSITION 0
#define MAXSTEPSPOSITION 1023

/**
	*	Abstract class for servo motors handlers that allow bus access to several motors
	@author Pablo Bustos - RoboLab - UEx
*/
class DynamixelSDK : public Handler
{
public:
	DynamixelSDK(RoboCompJointMotor::BusParams  *busParams, RoboCompJointMotor::MotorParamsList *params, QMutex *m);
	~DynamixelSDK();

	void initialize() throw(QString);
	RoboCompJointMotor::MotorParamsList getMotorParams( uchar motor){ return this->params[motor];};
	void setPosition(const QString &motor,  float  pos, float maxSpeed) throw(MotorHandlerUnknownMotorException,MotorHandlerErrorWritingToPortException);
	void setSyncPosition( const QVector<Handler::GoalPosition> & goals) throw(MotorHandlerErrorWritingToPortException);
	void getPosition(const QString &motor, float &pos) throw(MotorHandlerUnknownMotorException,MotorHandlerErrorWritingToPortException);
	bool getVelocity(const QString &motor, float &vel);
	bool setVelocity(const QString &motor, float vel);
	bool getPower(const QString & motor,float &pow);
	bool getMaxPosition(const QString &motor, float &pos);
	bool getMinPosition(const QString &motor, float &pos);
	bool setMaxPosition(const QString &motor, float pos);
	bool setMinPosition(const QString &motor, float pos);
	bool isMoving(const QString &motor);
	bool setId(const QString &motor,int id);
	bool powerOff(const QString &motor);
	bool powerOn(const QString &motor);
	bool setBaudrate(const QString &motor, int baud);
	bool getBaudrate(const QString &motor, int& baud);
	void update() throw(MotorHandlerUnknownMotorException,MotorHandlerErrorWritingToPortException);

	//Not implemented yet
	bool stop(uchar motor){motor=motor;return false;};
	bool reset(uchar motor){motor=motor;return false;};
	bool restoreDefValues(uchar motor){motor=motor;return false;};
	bool disablePWM(uchar motor){return false;};
	bool enablePWM(uchar motor){return false;};
	bool getReferenceVelocity(uchar  motor, float &vel){return false;};
	bool setReferenceVelocity(uchar  motor, float vel){return false;};
	bool getPGain(uchar motor, float &p){motor=motor;p=p;return false;};
	bool getDGain(uchar motor,float&d){motor=motor;d=d;return false;};
	bool getIGain(uchar motor,float&i){motor=motor;i=i;return false;};
	bool setPGain(uchar motor,float p){motor=motor;p=p;return false;};
	bool setDGain(uchar motor,float d){motor=motor;d=d;return false;};
	bool setIGain(uchar motor,float i){motor=motor;i=i;return false;};
	
	
	//Out of abstract Handler. Specific of Dynamixel
	bool getCWComplianceMargin( const QString &motor, int &m);
	bool getCCWComplianceMargin( const QString &motor, int &m);
	bool setBothComplianceMargins(const QString &motor, int m);
	bool getCWComplianceSlope( const QString &motor, int &m);
	bool getCCWComplianceSlope( const QString &motor, int &m);
	bool setBothComplianceSlopes(const QString &motor, int  m);
	bool setReturnDelayTime( const QString &motor, int t);
	bool getReturnDelayTime( const QString &motor, int &t);
	bool getPunch( const QString &motor, int &p);
	bool setPunch( const QString &motor, int p);
	bool getVoltageLimit( const QString &motor, int &v);
	bool setVoltageLimit( const QString &motor, int v);
	bool getStatusReturnLevel(const QString &motor,  int &level);
	bool setStatusReturnLevel(const QString &motor,  int level);
	bool getTemperature(const QString &motor,int &temperature);


	QMutex *h_mutex;	//hardware
	QMutex *w_mutex;	//worker
	QMutex *m_mutex;
	
private:

	void ping(char motor);

	RoboCompJointMotor::BusParams *busParams;
	RoboCompJointMotor::MotorParamsList *params;
	
	
	QHash<QString,int> motorsId;

	bool checkResponse();
};

#endif
#endif //COMPILE_DYNAMIXEL
