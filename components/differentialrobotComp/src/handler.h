#ifndef HANDLER_H
#define HANDLER_H

#include <QObject>
#include <QtCore>
#include <DifferentialRobot.h>
#include <GenericBase.h>

class Handler : public QObject
{
	
Q_OBJECT
public:
	Handler(){};
	~Handler(){};
	
	virtual bool setSpeedBase( float adv , float rot)=0;
	virtual bool stopBase()=0;
	virtual RoboCompGenericBase::TBaseState getBaseState()=0;
	virtual bool resetOdometer()=0;
	virtual bool setOdometer(RoboCompGenericBase::TBaseState bState)=0;
	virtual RoboCompDifferentialRobot::TMechParams getMechParams()=0;
	virtual void compute()=0;
	virtual void correctOdometer(float x, float z, float angle) = 0;
};

#endif
