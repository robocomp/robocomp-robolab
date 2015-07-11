
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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	mutex = new QMutex();
	
	values[SickLD::SICK_MAX_NUM_MEASUREMENTS] = {0};
	sector_step_angles[SickLD::SICK_MAX_NUM_MEASUREMENTS] = {0};
	num_values = 0;
	sector_start_ang = 90;
	sector_stop_ang = 270;
	pointsLaser.resize(sector_stop_ang-sector_start_ang+1);
	laserData.resize(sector_stop_ang-sector_start_ang+1);	
	sick_ld = new SickLD("192.168.187.204");
	try
	{
		sick_ld->Initialize();
	}
	catch(...)
	{
		cerr << "Initialize failed! Are you using the correct IP address?" << endl;
	}
	
//	try 
//	{
		/* Assign absolute and then relative time */
		//uint16_t new_sick_time = 0;
		//sick_ld.SetSickTimeAbsolute(1500,new_sick_time);
		//cout << "\tNew sick time: " << new_sick_time << endl;    
		//sick_ld.SetSickTimeRelative(-500,new_sick_time);
		//cout << "\tNew sick time: " << new_sick_time << endl;

		/* Configure the Sick LD sensor ID */
		//sick_ld.PrintSickGlobalConfig();
		//sick_ld.SetSickSensorID(16);
		//sick_ld.PrintSickGlobalConfig();

		/* Configure the sick motor speed */
		//sick_ld.PrintSickGlobalConfig();
		//sick_ld.SetSickMotorSpeed(10);
		//sick_ld.PrintSickGlobalConfig();

		/* Configure the sick scan resolution */
		//sick_ld.PrintSickGlobalConfig();
		//sick_ld.SetSickScanResolution(0.5);
		//sick_ld.PrintSickGlobalConfig();

		/* Configure all the global parameters */
		//double start_angle = 45;
		//double stop_angle = 315;
		//sick_ld.PrintSickGlobalConfig();
		//sick_ld.PrintSickSectorConfig();
		//sick_ld.SetSickGlobalParamsAndScanAreas(10,0.5,&start_angle,&stop_angle,1);
		//sick_ld.PrintSickGlobalConfig();
		//sick_ld.PrintSickSectorConfig();
//	}
//	catch(...)
//	{
//		cerr << "An error occurred!" << endl;
//	}

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	try
	{
		sick_ld->Uninitialize();
	}
	catch(...)
	{
		cerr << "Uninitialize failed!" << endl;
	}

	delete mutex;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
}

void SpecificWorker::compute()
{
	static int fps = 0;
	static QTime reloj = QTime::currentTime();
	int points = computePoints();
	pointsLaser.swap();
        if (reloj.elapsed() > 1000)
        {
           qDebug()<<"Read at:"<<points<<"ps";
           reloj.restart();
           fps=0;
       }
       fps++;
}

int SpecificWorker::computePoints()
{
	try {
		sick_ld->SetSickTempScanAreas(&sector_start_ang,&sector_stop_ang,1);
// 		sick_ld->PrintSickSectorConfig();
		for (unsigned int i = 0; i < 10; i++)
		{
			sick_ld->GetSickMeasurements(values,NULL,&num_values,NULL,sector_step_angles);
			double curAngle = sector_start_ang;
			for (unsigned int i = 0; i < num_values; i++)
			{
				pointsLaser[i].angle = curAngle;
				pointsLaser[i].dist = values[sector_step_angles[i]];
				curAngle++;
			} 
		}
	}
	catch(...)
	{
		cerr << "An error occurred!" << endl;
	}
	
	return num_values;
}

//////////////////////////////
/// SERVANT
//////////////////////////////
TLaserData SpecificWorker::getLaserData()
{
	pointsLaser.copy(laserData);
	return laserData;
}

LaserConfData SpecificWorker::getLaserConfData()
{

}

TLaserData SpecificWorker::getLaserAndBStateData(RoboCompDifferentialRobot::TBaseState &bState)
{

}