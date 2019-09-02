#if COMPILE_PLAYER==1
   
#ifndef PLAYERHANDLER_H
#define PLAYERHANDLER_H

#if defined(signals) && defined(QOBJECTDEFS_H) && !defined(QT_MOC_CPP)
#  undef signals
#  define signals signals
#endif

///lib player
#include <libplayerc++/playerc++.h>
namespace boost
{
  namespace signalslib = signals;
}
#if defined(signals) && defined(QOBJECTDEFS_H) && !defined(QT_MOC_CPP)
#  undef signals
#  define signals protected
#endif



#include <IceUtil/UUID.h>
#include <QObject>
#include <QtCore>
#include <QTimer>
#include <qlog/qlog.h>
#include "handler.h"
#include <math.h>
using namespace PlayerCc;

class PlayerHandler : public Handler
{
Q_OBJECT
private:
    PlayerClient    *client;  	
	Position2dProxy *pos;	
// 	gazebo::Client *client;
// 	gazebo::SimulationIface *simIface;
// 	gazebo::PositionIface *posIface;
    QMutex *mutex;
// 	
	RoboCompDifferentialRobot::TBaseState bState;
	RoboCompDifferentialRobot::TMechParams mechParams;	
// 
	float r00,r01,r10,r11,t0,t1,alfa;	//rotation and translation matrix
	float adv,rot;
	bool result;
	bool sendCommand;
	
public:
	PlayerHandler(RoboCompDifferentialRobot::TMechParams params);
	~PlayerHandler();
	void initMutex(QMutex *m){};

	//Speed
	bool setSpeedBase(float adv,float rot);
	bool stopBase();
	//Params
	RoboCompDifferentialRobot::TMechParams getMechParams();
	RoboCompDifferentialRobot::TBaseState getBaseState();
	//odometer
	bool resetOdometer();
	bool setOdometer(RoboCompDifferentialRobot::TBaseState _bState);
	void correctOdometer(float x, float z, float alpha) { bState.correctedX+=x; bState.correctedZ+=z; bState.correctedAlpha+=alpha; }
	void compute();
private:
	void readMechParams();	
	void computeBaseState();
	bool executeSpeed(float adv,float rot);

};

#endif

#endif

