/*
 *    Copyright (C)2019 by YOUR NAME HERE
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

/**
       \brief
       @author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <const.h>
#include <qjoystick/qjoystick.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
private:
	RoboCompJoyStick::JoyStickBufferedData joystickBufferedData; 
	RoboCompCommonBehavior::ParameterList params;
	std::shared_ptr<InnerModel> innerModel;
	QMutex *mutex;
	struct qjh_cfg_t
	{
		int32_t XMotionAxis;
		int32_t YMotionAxis;
		int32_t ZMotionAxis;
		int32_t maxAdvX, maxAdvZ;
		float maxRot;
		int32_t SampleRate;
	};
	struct qjh_joy_axis
	{
		float actualX;
		float actualY;
		float actualZ;
	};
	QTimer *jtimer; // Resend joy data to client
	qjh_joy_axis base_joy_axis;
	QJoyStick *joystick;
	qjh_cfg_t config;

	bool buttonPressed;
	bool sendSpeed;
public:
	SpecificWorker(MapPrx& mprx,bool startup_check=false);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	bool open();
	void JoyStick_writeJoyStickBufferedData(const RoboCompJoyStick::JoyStickBufferedData &gbd);
	void JoyStick_readJoyStickBufferedData(RoboCompJoyStick::JoyStickBufferedData &gbd);

public slots:
	void compute();
	void initialize(int period);
	void receivedJoyStickEvent( int value, int type, int number );
	void sendJoyStickEvent();

};

#endif
