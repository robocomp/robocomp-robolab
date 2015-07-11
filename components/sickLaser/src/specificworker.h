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

class SpecificWorker : public GenericWorker
{
Q_OBJECT
private:
 	RoboCompLaser::TLaserData laserData;
 	RoboCompLaser::LaserConfData laserConf;
 	RoboCompDifferentialRobot::DifferentialRobotPrx differentialrobot;
 	RoboCompDifferentialRobot::TBaseState bState;

public:
	RoboCompLaser::TLaserData getNewData();
	RoboCompLaser::LaserConfData getLaserConf();
	RoboCompDifferentialRobot::TBaseState getBaseData();


	SickLD *sick_ld = new SickLD("192.168.187.204");
	
	/* Define the data buffers */
	double values[SickLD::SICK_MAX_NUM_MEASUREMENTS];
	unsigned int num_values;

	/* Define the bounds for a single sector */
	double sector_start_ang;
	double sector_stop_ang;

	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
public slots:
	void compute(); 
private:
	
};

#endif