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
	ready = false;
}
/**
* \brief Default destructor
*/
SpecificMonitor::~SpecificMonitor()
{
	std::cout << "Destroying SpecificMonitor" << std::endl;
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
 *   (1) Ice parameters
 *   (2) Local component parameters read at start
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
	aux.editable = true;
	configGetString("",LASER_DRIVER_PROPERTY_NAME, aux.value, LASER_DRIVER_PROPERTY_DEFAULT);
	aux.type = "string";
	params[LASER_DRIVER_PROPERTY_NAME] = aux;
	configGetString("",LASER_DEVICE_PROPERTY_NAME, aux.value, LASER_DEVICE_PROPERTY_DEFAULT);
	params[LASER_DEVICE_PROPERTY_NAME] = aux;

	aux.type = "int";
	configGetString("",LASER_START_PROPERTY_NAME, aux.value, LASER_START_PROPERTY_DEFAULT);
	params[LASER_START_PROPERTY_NAME] = aux;
	configGetString("",LASER_END_PROPERTY_NAME, aux.value, LASER_END_PROPERTY_DEFAULT);
	params[LASER_END_PROPERTY_NAME] = aux;
	configGetString("",LASER_SKIP_PROPERTY_NAME, aux.value, LASER_SKIP_PROPERTY_DEFAULT);
	params[LASER_SKIP_PROPERTY_NAME] = aux;
	configGetString("",LASER_SAMPLERATE_PROPERTY_NAME, aux.value, LASER_SAMPLERATE_PROPERTY_DEFAULT);
	params[LASER_SAMPLERATE_PROPERTY_NAME] = aux;
	configGetString("",LASER_MAX_DEGREES_PROPERTY_NAME, aux.value, LASER_MAX_DEGREES_DEFAULT);
	params[LASER_MAX_DEGREES_PROPERTY_NAME] = aux;
	configGetString("",LASER_MAX_RANGE_PROPERTY_NAME, aux.value, LASER_MAX_RANGE_DEFAULT);
	params[LASER_MAX_RANGE_PROPERTY_NAME] = aux;
	configGetString("",LASER_MIN_RANGE_PROPERTY_NAME, aux.value, LASER_MIN_RANGE_DEFAULT);
	params[LASER_MIN_RANGE_PROPERTY_NAME] = aux;
	configGetString("",LASER_STATIC_CONF_PROPERTY_NAME, aux.value, LASER_STATIC_CONF_DEFAULT);
	params[LASER_STATIC_CONF_PROPERTY_NAME] = aux;
	configGetString("",LASER_CLUSTER_PROPERTY_NAME, aux.value, LASER_CLUSTER_PROPERTY_DEFAULT);
	params[LASER_CLUSTER_PROPERTY_NAME] = aux;	

	aux.type = "float";
	configGetString("",LASER_RESOLUTION_PROPERTY_NAME, aux.value, LASER_ANGLE_RESOLUTION_DEFAULT);
	params[LASER_RESOLUTION_PROPERTY_NAME] = aux;
	configGetString("",LASER_INITIAL_ANGLE_PROPERTY_NAME, aux.value, LASER_INITIAL_ANGLE_DEFAULT);
	params[LASER_INITIAL_ANGLE_PROPERTY_NAME] = aux;
	
	aux.type = "bool";
	configGetString("",LASER_TALKTOBASE_PROPERTY_NAME,aux.value,LASER_TALKTOBASE_DEFAULT);
	params[LASER_TALKTOBASE_PROPERTY_NAME] = aux;
}

//comprueba que los parametros sean correctos y los transforma a la estructura del worker
bool SpecificMonitor::checkParams(RoboCompCommonBehavior::ParameterList l)
{
	bool correct = true;
	return correct;
}

