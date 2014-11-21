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
#include "monitor.h"
/**
* \brief Default constructor
*/
Monitor::Monitor(Worker *_worker,Ice::CommunicatorPtr _communicator)
{
	worker = _worker;
	this->communicator = _communicator;
	period = 100;
	state = RoboCompCommonBehavior::Starting;
}
/**
* \brief Default destructor
*/
Monitor::~Monitor()
{

}

void Monitor::run()
{
	initialize();
	forever
	{
		rDebug("monitor run");
		this->sleep(500);
	}
}

/**
* \brief Get component execution state
* @return State Component state
*/
RoboCompCommonBehavior::State Monitor::getState()
{
	return state;
}

/**
* \brief Get worker period
* @return int Worker period in ms
*/
int Monitor::getPeriod()
{
	return period;
}
/**
* \brief Change worker period
* @param per Period in ms
*/
void Monitor::setPeriod(int _period)
{
	worker->setPeriod(_period);
	period =_period;
}
/**
* \brief Kill component
*/
void Monitor::killYourSelf()
{
	rDebug("Killing myself");
	this->exit();
	worker->killYourSelf();
	emit kill();
	
}
/**
* \brief Get Component time awake
* @return int Time alive in seconds
*/
int Monitor::timeAwake()
{
	return initialTime.secsTo(QTime::currentTime());
}
/**
* \brief Return components parameters
* @return  AttrList Configuration parameters list
*/
RoboCompCommonBehavior::ParameterList Monitor::getParameterList() 
{ 
	return config_params;
}
/**
* \brief Change configurations parameters to worker
* @param l Configuration parameters list
*/
void Monitor::setParameterList(RoboCompCommonBehavior::ParameterList l) 
{ 
	rInfo("Changing configuration params");
	sendParamsToWorker(l);
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
void Monitor::initialize()
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
bool Monitor::sendParamsToWorker(RoboCompCommonBehavior::ParameterList params)
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
		//return false;			//Change when implemented
		return true;
	}

}
///Local Component parameters read at start
///Reading parameters from config file or passed in command line, with Ice machinery
///We need to supply a list of accepted values to each call
void Monitor::readConfig(RoboCompCommonBehavior::ParameterList &params )
{
	//Read params from config file
	//Example
	    //RoboCompCommonBehavior::Parameter aux;
	    //aux.editable = true;
	    //configGetString( "DRobot.Device", aux.value,"/dev/ttyUSB0");
	    //params["DRobot.Device"] = aux;
	RoboCompCommonBehavior::Parameter aux;
	aux.editable = true;
	configGetString(LASER_DRIVER_PROPERTY_NAME, aux.value, LASER_DRIVER_PROPERTY_DEFAULT);
	aux.type = "string";
	params[LASER_DRIVER_PROPERTY_NAME] = aux;
	configGetString(LASER_DEVICE_PROPERTY_NAME, aux.value, LASER_DEVICE_PROPERTY_DEFAULT);
	params[LASER_DEVICE_PROPERTY_NAME] = aux;

	aux.type = "int";
	configGetString(LASER_START_PROPERTY_NAME, aux.value, LASER_START_PROPERTY_DEFAULT);
	params[LASER_START_PROPERTY_NAME] = aux;
	configGetString(LASER_END_PROPERTY_NAME, aux.value, LASER_END_PROPERTY_DEFAULT);
	params[LASER_END_PROPERTY_NAME] = aux;
	configGetString(LASER_SKIP_PROPERTY_NAME, aux.value, LASER_SKIP_PROPERTY_DEFAULT);
	params[LASER_SKIP_PROPERTY_NAME] = aux;
	configGetString(LASER_SAMPLERATE_PROPERTY_NAME, aux.value, LASER_SAMPLERATE_PROPERTY_DEFAULT);
	params[LASER_SAMPLERATE_PROPERTY_NAME] = aux;
	configGetString(LASER_MAX_DEGREES_PROPERTY_NAME, aux.value, LASER_MAX_DEGREES_DEFAULT);
	params[LASER_MAX_DEGREES_PROPERTY_NAME] = aux;
	configGetString(LASER_MAX_RANGE_PROPERTY_NAME, aux.value, LASER_MAX_RANGE_DEFAULT);
	params[LASER_MAX_RANGE_PROPERTY_NAME] = aux;
	configGetString(LASER_MIN_RANGE_PROPERTY_NAME, aux.value, LASER_MIN_RANGE_DEFAULT);
	params[LASER_MIN_RANGE_PROPERTY_NAME] = aux;
	configGetString(LASER_STATIC_CONF_PROPERTY_NAME, aux.value, LASER_STATIC_CONF_DEFAULT);
	params[LASER_STATIC_CONF_PROPERTY_NAME] = aux;
	
	aux.type = "float";
	configGetString(LASER_RESOLUTION_PROPERTY_NAME, aux.value, LASER_ANGLE_RESOLUTION_DEFAULT);
	params[LASER_RESOLUTION_PROPERTY_NAME] = aux;
	configGetString(LASER_INITIAL_ANGLE_PROPERTY_NAME, aux.value, LASER_INITIAL_ANGLE_DEFAULT);
	params[LASER_INITIAL_ANGLE_PROPERTY_NAME] = aux;
	
	aux.type = "bool";
	configGetString(LASER_TALKTOBASE_PROPERTY_NAME,aux.value,LASER_TALKTOBASE_DEFAULT);
	params[LASER_TALKTOBASE_PROPERTY_NAME] = aux;
}

//comprueba que los parametros sean correctos y los transforma a la estructura del worker
bool Monitor::checkParams(RoboCompCommonBehavior::ParameterList l)
{
	bool correct = true;
	//Check parameters
	//Example
// 	    if(l["DRobot.Handler"].value != "Robex" and l["DRobot.Handler"].value != "Gazebo" and l["DRobot.Handler"].value != "Player")
// 		    correct = false;	
	
	
	//copy parameters
// 	if(correct)
// 		config_params = l;
	return correct;
}


//Ice Methods to read from file
bool Monitor::configGetString( const std::string name, std::string &value,  const std::string default_value, QStringList *list)
{
	value = communicator->getProperties()->getProperty( name );
	if ( value.length() == 0)
	{
		value = default_value;
		return true;
	}
	if(list != NULL)
	{
		if (list->contains(QString::fromStdString(value)) == false)
		{
			qFatal("Reading config file: %s is not a valid string", name.c_str());
			rError("Reading config file:"+name+" is not a valid string");
		}
	}
	std::cout << name << " " << value << std::endl;
	return true;
}

