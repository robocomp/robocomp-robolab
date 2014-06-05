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
#ifndef HANDLER_H
#define HANDLER_H

#include <QObject>
#include "servo.h"
#include "TSQHash.h"

/**
	*	Abstract class for servo motors handlers that allow bus access to several motors
	@author Pablo Bustos - Robolab - Uex
*/

class MotorHandlerUnknownMotorException: public std::exception
{
  public:
	MotorHandlerUnknownMotorException()
	{
	  s = "Illegal Motor Number: "; 
	}; 
	MotorHandlerUnknownMotorException( const std::string &c)
	{ 
	  s = "Illegal Motor Number: " + c; 
	}
	virtual ~MotorHandlerUnknownMotorException() throw(){};
	void say(const std::string c)
	{
	  s+=c;
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
	};
	~MotorHandlerErrorWritingToPortException() throw(){};
	void say(const std::string c)
	{
	  s+=c;
	}
  private:
	std::string s;
};

class Handler
{
public:
    Handler(){h_mutex = new QMutex();};
    ~Handler(){};
	
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
	  float speed;
	  float maxAcc;
	};
	

	virtual void initialize() throw(QString) =0;
	virtual void setPosition(const QString &motor,  float  pos, float maxSpeed) throw(MotorHandlerUnknownMotorException,MotorHandlerErrorWritingToPortException) =0;
	virtual void getPosition(const QString &motor,float &pos) throw(MotorHandlerUnknownMotorException,MotorHandlerErrorWritingToPortException) =0;
	virtual bool getVelocity(const QString &motor,float &vel)=0;
	virtual bool setVelocity(const QString &motor,float vel)=0;
	
	virtual bool getReferenceVelocity(uchar  motor,float &vel)=0;
	virtual bool setReferenceVelocity(uchar  motor,float vel)=0;
	
	virtual bool getTemperature(const QString &moto, int &temperature)=0;
	virtual bool getPower(const QString &motor,float &pow)=0;
	virtual bool getMaxPosition(const QString &motor,float &pos)=0;
	virtual bool getMinPosition(const QString &motor,float &pos)=0;
	virtual bool setMaxPosition(const QString &motor,float pos)=0;
	virtual bool setMinPosition(const QString &motor,float pos)=0;
	virtual bool isMoving(const QString &motor)=0;
	virtual bool getPGain(uchar  motor,float &p)=0;
	virtual bool getDGain(uchar  motor,float &d)=0;
	virtual bool getIGain(uchar  motor,float &i)=0;
	virtual bool setPGain(uchar  motor,float p)=0;
	virtual bool setDGain(uchar  motor,float d)=0;
	virtual bool setIGain(uchar  motor,float i)=0;
	virtual bool disablePWM(uchar  motor)=0;
	virtual bool enablePWM(uchar  motor)=0;
	virtual bool setId(const QString &motor, int id)=0;
	virtual bool reset(uchar  motor)=0;
	virtual bool restoreDefValues(uchar  motor)=0;
	virtual bool stop(uchar  motor)=0;
	virtual bool powerOff(const QString &motor)=0;
	virtual bool powerOn(const QString &motor)=0;
	virtual bool setBaudrate(const QString &motor, int baud)=0;
	virtual void setSyncPosition( const QVector<GoalPosition> & goalList ) throw(MotorHandlerErrorWritingToPortException) = 0;
	virtual void update() throw(MotorHandlerUnknownMotorException,MotorHandlerErrorWritingToPortException) = 0;
	
	TSQHash<QString, Servo*> motors;
	QMutex *h_mutex;
};

#endif

