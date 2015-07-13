/*
 *    Copyright (C) 2015 by YOUR NAME HERE
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

#include <QtCore>
#include <stdint.h>
#include <qlog/qlog.h>
#include <Laser.h>
#include <string>
#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include "ConfigFile.h"
#include "SickLD.hh"

#define INVALID_OPTION_STRING        "   Invalid option!!! :o(" 
#define PROMPT_STRING                                   "ld?> " 

/* Config file parameters */
#define CONFIG_OPT_MOTOR_SPD_STR          "SICK_LD_MOTOR_SPEED"
#define CONFIG_OPT_SCAN_AREA_STR           "SICK_LD_SCAN_AREAS"
#define CONFIG_OPT_SCAN_RES_STR       "SICK_LD_SCAN_RESOLUTION"

using namespace std;
using namespace SickToolbox;


template <class T> class DoubleBuffer
{
	
public:
	DoubleBuffer()
	{
		size = -1;
	}
	void resize(int size_)
	{
		bufferMutex.lock();
			bufferA.resize(size_);
			writer = &bufferA;
			bufferB.resize(size_);
			reader = &bufferB;
			size = size_;
		bufferMutex.unlock();
	}
	void swap()
	{
		bufferMutex.lock();
			writer->swap(*reader);
		bufferMutex.unlock();
	}

	inline typename T::value_type& operator[](int i)
	{
		return (*writer)[i];
	}

	void copy(T &points)
	{
		bufferMutex.lock();
			points.resize(size);
			points = *reader;
		bufferMutex.unlock();
	}

	
	
	int getSize() { return size; }
private:
	QMutex bufferMutex;
	T bufferA, *writer, *reader, bufferB;

	int size;
	
};


class SpecificWorker : public GenericWorker
{
Q_OBJECT
private:
	RoboCompLaser::LaserConfData laserDataConf;
	DoubleBuffer<RoboCompLaser::TLaserData> pointsLaser;
public:
	SickLD *sick_ld;

	double values[SickLD::SICK_MAX_NUM_MEASUREMENTS];
	unsigned int echo[SickLD::SICK_MAX_NUM_MEASUREMENTS];
	unsigned int num_values;
	unsigned int sector_step_angles[SickLD::SICK_MAX_NUM_MEASUREMENTS];

	double sector_start_ang;
	double sector_stop_ang;

	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	int computePoints();
	
	TLaserData getLaserData();
	LaserConfData getLaserConfData();
	TLaserData getLaserAndBStateData(RoboCompDifferentialRobot::TBaseState &bState);
public slots:
	void compute(); 	
};

#endif