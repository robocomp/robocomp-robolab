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
#include <QtSerialPort/QSerialPort>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	QPointF readData(QSerialPort &serial);
	void compute_initial_pose(int ntimes);
    float degreesToRadians(const float angle_);
	int bitsToInt( const unsigned char* bits, uint init, bool little_endian = true );
	//Ice interface
	FullPose FullPoseEstimation_getFullPose();
	void FullPoseEstimation_setInitialPose(float x, float y, float z, float rx, float ry, float rz);
	
public slots:
	void compute();
	void initialize(int period);
//Specification slot methods State Machine
	void sm_compute();
	void sm_initialize();
	void sm_finalize();

//--------------------
private:
	std::shared_ptr<InnerModel> innerModel;
    RoboCompCommonBehavior::ParameterList params;
	QSerialPort left_device, right_device;
	int ndevices = 0;
	int left_offset;
	int right_offset;
	float xPos;
	float zPos;
	float ryPos;

};

#endif
