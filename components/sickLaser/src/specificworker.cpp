
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
	
	for (int i=0; i<SickLD::SICK_MAX_NUM_MEASUREMENTS; i++)
	{
		values[i] = 0;
		sector_step_angles[i] = 0;
		echo[i]=0;
	}
	num_values = 0;
	sector_start_ang = 90;
	sector_stop_ang = 270;
	pointsLaser.resize(sector_stop_ang-sector_start_ang+1);
	sick_ld = new SickLD("192.168.187.204");
	
	
	try
	{		
		sick_ld->Initialize();
	}
	catch(...)
	{
		cerr << "Initialize failed! Are you using the correct IP address?" << endl;
	}
	
	sector_start_ang = 90;
	sector_stop_ang = 270;
	sick_ld->SetSickGlobalParamsAndScanAreas(5, 1, &sector_start_ang,&sector_stop_ang, 1);
// 	sick_ld->SetSickTempScanAreas(&sector_start_ang,&sector_stop_ang,1);
	sick_ld->PrintSickSectorConfig();

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
	try
	{
			sick_ld->GetSickMeasurements(values,echo, &num_values);//,NULL,sector_step_angles);
			for (unsigned int i = 0; i < num_values; i++)
			{
// 				laserPolarData[i] = (double)values[i]/2;
				pointsLaser[i].angle = (i + 90.)*M_PI/180.f;
				pointsLaser[i].dist = (double)values[i]*1000;
			} 
	}
	catch(...)
	{
		cerr << "An error occurred!" << endl;
	}
	
	return num_values;
}


void getThetas(beg_ang,end_ang,res_ang)
{
	if beg_ang > end_ang, end_ang = end_ang+360; end
		theta = mod((beg_ang:res_ang:end_ang)',360).*pi/180;

}



//////////////////////////////
/// SERVANT
//////////////////////////////
TLaserData SpecificWorker::getLaserData()
{
	TLaserData laserData;
	pointsLaser.copy(laserData);
	return laserData;
}

LaserConfData SpecificWorker::getLaserConfData()
{
	return laserDataConf;
}

TLaserData SpecificWorker::getLaserAndBStateData(RoboCompDifferentialRobot::TBaseState &bState)
{
	TLaserData laserData;
	pointsLaser.copy(laserData);
	return laserData;
}
