
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
#include "specificmonitor.h"
/**
* \brief Default constructor
*/
SpecificMonitor::SpecificMonitor(GenericWorker *_worker,Ice::CommunicatorPtr _communicator):GenericMonitor(_worker, _communicator)
{

}
/**
* \brief Default destructor
*/
SpecificMonitor::~SpecificMonitor()
{

}

void SpecificMonitor::run()
{
	initialize();
	forever
	{
		rDebug("specific monitor run");
		this->sleep(period);
	}
}


/**
 * \brief Reads components parameters and checks set integrity before signaling the Worker thread to start running
 * There can be four (4) types of parameteres:
 *		(1) Ice parameters
 *		(2) Nexus (configuration) parameters	
 *		(3) Local component parameters read at start
 *		(4) Local parameters read from other running component
 *
 */
void SpecificMonitor::initialize()
{
	rInfo("Starting monitor ...");
	initialTime=QTime::currentTime();
	RoboCompCommonBehavior::ParameterList params;
	readConfig(params );
	if(!sendParamsToWorker(params))
	{
		rError("Error reading config parameters. Exiting");
		killYourSelf();
	}
	state = RoboCompCommonBehavior::Running;
}
bool SpecificMonitor::sendParamsToWorker(RoboCompCommonBehavior::ParameterList params)
{
	if(checkParams(params))
	{
		//Set params to worker
		worker->setParams(params);
		return true;
	}
	else
	{
		rError("Incorrect parameters");
		return false;
	}
}
///Local Component parameters read at start
///Reading parameters from config file or passed in command line, with Ice machinery
///We need to supply a list of accepted values to each call
void SpecificMonitor::readConfig(RoboCompCommonBehavior::ParameterList &params )
{
	RoboCompCommonBehavior::Parameter aux;
	configGetString( "Faulhaber.NumMotors", aux.value, "0" );	
	int num_motors=0;
	num_motors = QString::fromStdString(aux.value).toInt();
	if(num_motors <= 0) 
	  qFatal("Monitor::initialize - Zero motors found. Exiting..." );

	params["Faulhaber.NumMotors"] = aux;
	configGetString( "Faulhaber.Device", aux.value, "/dev/ttyUSB0" );	
	params["Faulhaber.Device"] = aux;

	configGetString( "Faulhaber.BaudRate", aux.value, "" );
	params["Faulhaber.BaudRate"] = aux;
	
	configGetString( "Faulhaber.BasicPeriod", aux.value, "100" );
	params["Faulhaber.BasicPeriod"] = aux;
	
	std::string paramsStr;
	for (int i=0; i<num_motors; i++)
	{
		std::string s= QString::number(i).toStdString();
		configGetString( "Faulhaber.Params_" + s, aux.value , "");
		params["Faulhaber.Params_" + s] = aux;
		QStringList list = QString::fromStdString(aux.value).split(",");
		if (list.size() != 9) qFatal("Error reading motor. Only %d parameters for motor %d.", list.size(), i);
		
		aux.value=list[0].toStdString();
		params["Faulhaber.Params_" + s +".name"]= aux;
		aux.value=list[1].toStdString();
		params["Faulhaber.Params_" + s +".busId"]= aux;
		aux.value=list[2].toStdString();
		params["Faulhaber.Params_" + s +".minPos"]= aux;
		aux.value=list[3].toStdString();
		params["Faulhaber.Params_" + s +".maxPos"]= aux;
		aux.value=list[4].toStdString();
		params["Faulhaber.Params_" + s +".zeroPos"]= aux;
		aux.value=list[5].toStdString();	
		params["Faulhaber.Params_" + s +".maxVelocity"]= aux;
		aux.value=list[6].toStdString();	
		params["Faulhaber.Params_" + s +".stepsRange"]= aux;
		aux.value=list[7].toStdString();
		params["Faulhaber.Params_" + s +".unitsRange"]= aux;
		aux.value=list[8].toStdString();	
		params["Faulhaber.Params_" + s +".offset"]= aux;
	}
}

//comprueba que los parametros sean correctos y los transforma a la estructura del worker
bool SpecificMonitor::checkParams(RoboCompCommonBehavior::ParameterList params)
{
	bool correct = true;
	if(QString::fromStdString(params["Faulhaber.NumMotors"].value).toFloat() < 1)
		correct = false;	

	return correct;
}
