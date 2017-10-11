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
	ready = true;
	forever
	{
		//rDebug("specific monitor run");
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
	readPConfParams(params);
	readConfig(params);
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
		if(worker->setParams(params)) 
			return true;
	}
	else
	{
		rError("Incorrect parameters");
	}
	return false;

}
///Local Component parameters read at start
///Reading parameters from config file or passed in command line, with Ice machinery
///We need to supply a list of accepted values to each call
void SpecificMonitor::readConfig(RoboCompCommonBehavior::ParameterList &params )
{
 	RoboCompCommonBehavior::Parameter aux;
 	aux.editable = false;
 	configGetString("", "CameraV4L.Device0.Name", aux.value, "default");
 	params["CameraV4L.Device0.Name"] = aux;

	aux.editable = false;
	configGetString("",  "CameraV4L.Device0.FPS", aux.value, "15");
	if( aux.value != "30" and aux.value != "15" and aux.value != "10" and aux.value != "5")
	{
		std::cout << __FUNCTION__ << "Warning. Wrong FPS value. Using default 15" << std::endl;
		aux.value = "15";
	}
	params["CameraV4L.Device0.FPS"] = aux;

	aux.editable = false;
	configGetString("",  "CameraV4L.Device0.Width", aux.value, "640");
	if(aux.value != "1920" and aux.value != "1280" and aux.value != "640" and aux.value != "320" and aux.value != "160")
	{
		std::cout << __FUNCTION__ << "Warning. Wrong Width value. Using default 640" << std::endl;
		aux.value = "640";
	}
	params["CameraV4L.Device0.Width"] = aux;
	
	aux.editable = false;
	configGetString("",  "CameraV4L.Device0.Height", aux.value, "480");
	if( aux.value!="1080" and aux.value!="720"  and aux.value != "480" and aux.value != "240" and aux.value != "120")
	{
		std::cout << __FUNCTION__ << "Warning. Wrong Height value. Using default 480" << std::endl;
		aux.value = "480";
	}
	params["CameraV4L.Device0.Height"] = aux;
	
}


//comprueba que los parametros sean correctos y los transforma a la estructura del worker
bool SpecificMonitor::checkParams(RoboCompCommonBehavior::ParameterList l)
{
	bool correct = true;
	return correct;
}

