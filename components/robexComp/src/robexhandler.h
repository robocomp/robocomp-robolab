/****************************************************************************
#     Copyright (C) 2008 Robolab-UEx                                        #
#                                                                           #
# This program is free software; you can redistribute it and/or modify      #
# it under the terms of the GNU General Public License as published by      #
# the Free Software Foundation; either version 2 of the License, or         #
# (at your option) any later version.                                       #
#                                                                           #
# This program is distributed in the hope that it will be useful,           #
# but WITHOUT ANY WARRANTY; without even the implied warranty of            #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             #
# GNU General Public License for more details.                              #
#                                                                           #
# You should have received a copy of the GNU General Public License         #
# along with this program; if not, write to the Free Software               #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA #
#                                                                           #
****************************************************************************/
#ifndef ROBEXHANDLER_H
#define ROBEXHANDLER_H


#include "handler.h"
#include <q4serialport/q4serialport.h>
#include <qlog/qlog.h>
// Timeouts
#define COMMAND_RESPONSE_TIMEOUT    100
#define COMMAND_FREE_MOTORS_CHECK_INTERVAL  3000
#define COMMAND_FREE_MOTORS_TIMEOUT         5000

// Set Base Speed commands: adv (mm/s) rot(rads/seg)  C0:SAaasBbb:0*
#define MOTION_BASE_COMMAND_STRING "C0:"
#define MOTION_BASE_COMMAND_CRC    "0*"
// Set Right Wheel Speed: Motor motion commands
#define MOTION_RIGHT_COMMAND_STRING "A1:"
#define MOTION_RIGHT_COMMAND_CRC    "1*"
// Set Left Wheel Speed: Motor motion commands
#define MOTION_LEFT_COMMAND_STRING "B1:"
#define MOTION_LEFT_COMMAND_CRC    "1*"
// Set Both Wheels Speed: Motor motion commands C1:SAaasBbb:1*
#define MOTION_COMMAND_STRING "C1:"
#define MOTION_COMMAND_CRC    "1*"
// Set motors state command
#define MOTION_MOTORS_STATE_COMMAND_STRING  "C9:"
#define MOTION_MOTORS_STATE_COMMAND_CRC     "9*"
// Motors States
#define MOTORS_STATE_NORMAL 0
#define MOTORS_STATE_FREE   1
#define MOTORS_STATE_BREAK  2
// Get Base data command: Reductora, Radio, Separación entre ruedas
#define BASEDATA_GET_COMMAND_STRING "D0:"
#define BASEDATA_GET_COMMAND_CRC    "0*"
#define BASEDATA_DATA_SIZE   3+2+1+2+1+3+2
#define BASEDATA_GETGEAR_DATASIZE 2
#define BASEDATA_GETRADIO_DATASIZE 2
#define BASEDATA_GETBASELINE_DATASIZE 3
// Odometer commands
#define ODOMETER_RESET_STRING "D1:"
#define ODOMETER_RESET_CRC "1*"
#define ODOMETER_GETPOSWHEELS_STRING "D2:"
#define ODOMETER_GETPOSWHEELS_CRC "2*"
// 3 bytes per parameter
#define ODOMETER_GETPOSWHEELS_DATA_SIZE   3 
#define ODOMETER_GETSPEEDWHEELS_STRING "D3:"
#define ODOMETER_GETSPEEDWHEELS_CRC "1*"
#define ODOMETER_GETSPEEDWHEELS_DATA_SIZE   3
#define ODOMETER_GETPOSBASE_STRING "E1:"
#define ODOMETER_GETPOSBASE_CRC "1*"
#define ODOMETER_GETPOSBASE_DATA_SIZE  10 
// Odometer commands Version binario
#define ODOMETER_GETPOSWHEELS_B_STRING "D4:"
#define ODOMETER_GETPOSWHEELS_B_CRC "4*"
#define ODOMETER_GETPOSWHEELS_B_DATA_SIZE   4

//Encoder steps per turn HP
#define ENCODER_STEPS 2000


/**
 *\brief This class is an handler for mobile robot Robex.
 * This object interacts with the Robex-AVR, a microcontroller based device that controls the motors of the robots using PID loops
 * @author Robolab staff
*/
class RobexHandler: public Handler
{
Q_OBJECT
private:
	//Structure that holds mechanical parameters
	RoboCompDifferentialRobot::TMechParams mechParams;

	QSerialPort MotionDevice;
	QTime LastCmdTimestamp;                             // Timestamp of last command received
	bool AreMotorsActivated;

	int lant,rant;                                      //t-1 values of wheels pos used in computePosBase. ResetOdometer zeroes them	
	float adv,rot;
	float advE,rotE;
	bool commandToSend;
	bool result;
    
	QMutex *bstate_mutex,*speed_mutex;
	RoboCompGenericBase::TBaseState bState;
	unsigned char stringEnviado[200], stringRecibido[200];
	int numEnviado, numRecibido;


public:
	RobexHandler(RoboCompDifferentialRobot::TMechParams params);
	~RobexHandler();
//accesible
	//odometer
	bool resetOdometer();	
	bool setOdometer(RoboCompGenericBase::TBaseState bState);
	void correctOdometer(float x, float z, float alpha);
	//speed
	bool stopBase();
	bool setSpeedBase(float adv,float rot);
	RoboCompGenericBase::TBaseState getBaseState();
	RoboCompDifferentialRobot::TMechParams getMechParams();
	void compute();

private: //internal
	bool executeSpeed(float adv,float rot);
	bool getBaseData();	//read mechanical parameters
	bool getPosWheels(int &izq, int &der);
	bool setSpeedWheels ( float left, float right );
	bool setMotorsState(int);
	void computePosBase();
	void computeAutonomy();
	void checkMotorsState();
	bool ascii2dec(char*, int, int&);	
	//command
	bool sendCommand( QString , char*, int);

};
#endif
