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

#ifndef DUNKER_H
#define DUNKER_H


#include <q4serialport/q4serialport.h>
#include <QtCore>
#include "iostream"
#include "JointMotor.h"
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


//  
#define NO_RESPONSE
#define READ_RESPONSE
#define ALL_RESPONSE


struct DunkerParams
{
		int maxPosErr;
		int accV;
		int accT;
		int decV;
		int decT;
		float gearRev;
		float motorRev;
		int posCurLim;
		int negCurLim;
		int curPeak;
		int curContin;
		int curTime;
		int encoderRes;
		bool setPID;
		int velKp;
		int velKi;
		int velKd;
};

class MotorHandlerUnknownMotorException: public std::exception
{
public:
	MotorHandlerUnknownMotorException()
	{
		s = "Illegal Motor Number: "; 
	} 
	MotorHandlerUnknownMotorException( const std::string &c)
	{ 
		s = "Illegal Motor Number: " + c; 
	}
	virtual ~MotorHandlerUnknownMotorException() throw(){};
	void say(const std::string c)
	{
		s += c;
	}
private:
	std::string s;
};

class MotorHandlerErrorWritingToPortException : public std::exception
{
public:
	MotorHandlerErrorWritingToPortException()
	{
		s = "Error writing to port: "; 
	}
	MotorHandlerErrorWritingToPortException(const std::string &c)
	{
		s = "Error writing to port: " + c; 
	}
	~MotorHandlerErrorWritingToPortException() throw(){};
	void say(const std::string c)
	{
		s+=c;
	}
private:
	std::string s;
};

/**
	@author Robolab <authormail>
*/

class Dunkermotoren 
{
typedef char* Packet;
public:
    Dunkermotoren( RoboCompJointMotor::BusParams  *busParams, RoboCompJointMotor::MotorParamsList *params, QHash<int, DunkerParams> *dunkerParams, QMutex *m);
    virtual ~Dunkermotoren();

	struct GoalPosition
	{
	  QString name;
	  uchar busDir;
	  float position; //rads
	  float maxVel;  //rads/sg
	  GoalPosition(){ name = "", busDir = 0; position = 0; maxVel = 0;};
	  GoalPosition( const QString & n, uchar d, float p, float mv): name(n), busDir(d), position(p), maxVel(mv) {};
	};
	
	struct GoalVelocity
	{
	  QString name;
	  uchar busDir;
	  float velocity;
	  float maxAcc;
	  GoalVelocity(){ name = "";  busDir = 0; velocity = 0; maxAcc = 0;};
	  GoalVelocity( const QString & n, uchar d, float vel, float acc): name(n), busDir(d), velocity(vel), maxAcc(acc) {};
	};
	
	QHash<QString, Servo*> motors;
private:
	int devHandler;
	
	RoboCompJointMotor::BusParams *busParams;
	RoboCompJointMotor::MotorParamsList *params;
	QHash<int,DunkerParams> *dunkerParams;
	QMutex *memory_mutex;
	QMutex *hardware_mutex;
public:
	virtual void initialize() throw(QString);
	RoboCompJointMotor::MotorParamsList getMotorParams( uchar motor){ return this->params[motor];};	
	virtual void setPosition(const QString &motor,  float  pos, float maxSpeed) throw(MotorHandlerUnknownMotorException,MotorHandlerErrorWritingToPortException);
	virtual void setSyncPosition( const QVector<Dunkermotoren::GoalPosition> & goals) throw(MotorHandlerErrorWritingToPortException);
	virtual void setSyncRelativePosition( const QVector<Dunkermotoren::GoalPosition> & goals) throw(MotorHandlerErrorWritingToPortException);
	virtual void getPosition(const QString &motor,float &pos) throw(MotorHandlerUnknownMotorException,MotorHandlerErrorWritingToPortException);
	void getSyncPosition(const QList<QString> &in_motors,int *pos) throw(MotorHandlerUnknownMotorException,MotorHandlerErrorWritingToPortException);
	void getSyncVelocities(const QList<QString> &in_motors,int *vels) throw(MotorHandlerUnknownMotorException,MotorHandlerErrorWritingToPortException);
	void getSyncStatusWords(const QList<QString> &in_motors,int *status_words) throw(MotorHandlerUnknownMotorException,MotorHandlerErrorWritingToPortException);
	void getSyncVoltages(const QList<QString> &in_motors,int *voltages) throw(MotorHandlerUnknownMotorException,MotorHandlerErrorWritingToPortException);
	void getSyncVelKps(const QList<QString> &in_motors,int *kps) throw(MotorHandlerUnknownMotorException,MotorHandlerErrorWritingToPortException);
	void getSyncVelKis(const QList<QString> &in_motors,int *kis) throw(MotorHandlerUnknownMotorException,MotorHandlerErrorWritingToPortException);
	void getSyncVelKds(const QList<QString> &in_motors,int *kds) throw(MotorHandlerUnknownMotorException,MotorHandlerErrorWritingToPortException);
	virtual bool getVelocity(uchar motor, float &vel);
	virtual bool setVelocity(const QString &motor, float  vel);
	virtual bool getStatusWord( uchar motor, int& status_word );
	virtual bool getErrorRegister( uchar motor, int &erro_register);
	void setZeroPos(const QString &motor);
	void setSyncZeroPos();
	
	virtual bool getReferenceVelocity(uchar  motor, float &vel);
	virtual bool setReferenceVelocity(const QString &motor, float vel);
	
	virtual bool setSyncReferenceVelocity( const QVector<Dunkermotoren::GoalVelocity> & goals) throw(MotorHandlerErrorWritingToPortException);
		
	virtual bool getPower(uchar motor, float &pow);
	virtual bool getMaxPosition(uchar motor, float &pos);
	virtual bool getMinPosition(uchar motor, float &pos);
	virtual bool setMaxPosition(const QString &motor,float pos);
	virtual bool setMinPosition(const QString &motor,float pos);
	virtual bool isMoving(const QString &motor);
	virtual bool getPGain(uchar motor, float &p){motor=motor;p=p;return false;};
	virtual bool getDGain(uchar motor, float &d){motor=motor;d=d;return false;};
	virtual bool getIGain(uchar motor, float &i){motor=motor;i=i;return false;};
	virtual bool setPGain(uchar motor, float p){motor=motor;p=p;return false;};
	virtual bool setDGain(uchar motor, float d){motor=motor;d=d;return false;};
	virtual bool setIGain(uchar motor, float i){motor=motor;i=i;return false;};
	virtual bool disablePWM(uchar motor){return false;};
	virtual bool enablePWM(uchar motor){return false;};
	virtual bool setId(uchar motor, int id);
	virtual bool reset(uchar motor);
	virtual bool restoreDefValues(uchar motor){motor=motor;return false;};
	virtual bool stop(uchar motor){motor=motor;return false;};
	virtual bool powerOff(uchar motor);
	virtual bool powerOn(uchar motor);
	virtual bool setBaudrate(uchar motor, int baud);
	virtual bool getBaudrate(uchar motor, int& baud);
	virtual void update() throw(MotorHandlerUnknownMotorException, MotorHandlerErrorWritingToPortException);
	

	//Out of abstract Handler. Specific of Dunkermotoren
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
	bool getVoltageLimit(const QString &motor, int & v);
	bool setVoltageLimit( uchar motor, int v);
	bool getStatusReturnLevel(uchar motor,  int &level);
	bool setStatusReturnLevel(uchar motor,  int &level);
	bool getAcceleration(const QString &motor, int& deltaV, int& deltaT );
	bool getRevolutions(const QString &motor, int& motorRevol, int& shaftRevol );
	
	//Not implemented yet
// 	bool getTemperature(uchar motor,  int &level);
// 	bool setTemperature(uchar motor,  int &level);
// 	bool getTemperatureLimit(uchar motor,  int &level);
// 	bool setTemperatureLimit(uchar motor,  int &level);
// 	bool getMaxTorque(uchar motor,  int &level);
// 	bool setMaxTorque(uchar motor,  int &level);
// 	bool getVoltage(uchar motor,  int &level);
// 	bool setVoltage(uchar motor,  int &level);
// 	bool get CWAngleLimit();
// 	bool set CWAngleLimit();
// 	bool get CCWAngleLimit();
// 	bool set CCWAngleLimit();
	
	
};

#endif

