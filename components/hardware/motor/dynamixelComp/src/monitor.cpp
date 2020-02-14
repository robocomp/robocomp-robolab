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

#include <fstream>

#include "monitor.h"
/**
* \brief Default constructor
*/
Monitor::Monitor( Worker *_worker, Ice::CommunicatorPtr _communicator )
{
	ready = false;
	period = 100;
	worker = _worker;
	this->communicator = _communicator;
	state = RoboCompCommonBehavior::Starting;	
}
/**
* \brief Default destructor
*/
Monitor::~Monitor()
{

}
/**
 * \brief Thread method
 */
void Monitor::run()
{
	initialize();
	ready = true;
	forever
	{
		rDebug("Monitor::run()");
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
* @return  ParameterList Configuration parameters list
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
 * \brief Reads components parameters and checks set integrity before signaling the Worker thread to start runningç
 * There can be four (4) types of parameteres:
 *		(1) Ice parameters
 *		(2) Nexus (configuration) parameters	
 *		(3) Local component parameters read at start
 *		(4) Local parameters read from other running component
 *
 */
bool Monitor::initialize( )
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
	return true;
}

///Local Component parameters read at start
///Reading parameters from config file or passed in command line, with Ice machinery
///We need to supply a list of accepted values to each call
void Monitor::readConfig(RoboCompCommonBehavior::ParameterList &params )
{
	RoboCompCommonBehavior::Parameter aux;
	aux.editable = true;
	configGetString( "Dynamixel.NumMotors", aux.value, "" );	
	int num_motors=0;
	num_motors = QString::fromStdString(aux.value).toInt();
	if(num_motors <= 0) 
	  qFatal("Monitor::initialize - Zero motors found. Exiting..." );

	params["Dynamixel.NumMotors"] = aux;
	configGetString( "Dynamixel.Device", aux.value, "" );	
	params["Dynamixel.Device"] = aux;
	
	configGetString( "Dynamixel.BaudRate", aux.value, "115200" );
	params["Dynamixel.BaudRate"] = aux;
	
	configGetString( "Dynamixel.BasicPeriod", aux.value, "100" );
	params["Dynamixel.BasicPeriod"] = aux;

	configGetString( "Dynamixel.SDK", aux.value, "false" );
	params["Dynamixel.SDK"] = aux;
	
	std::string paramsStr;
	for (int i=0; i<num_motors; i++)
	{
		std::string s= QString::number(i).toStdString();
		configGetString( "Dynamixel.Params_" + s, aux.value , "");
		params["Dynamixel.Params_" + s] = aux;
		QStringList list = QString::fromStdString(aux.value).split(",");
		if (list.size() < 9) qFatal("Error reading motor. Only %d parameters for motor %d. Expecting 9.", list.size(), i);
		else if (list.size() > 9) qFatal("Error reading motor. More than 9 (%d) parameters for motor %d.", list.size(), i);
		aux.value=list[0].toStdString();
		params["Dynamixel.Params_" + s +".name"]= aux;
		aux.value=list[1].toStdString();
		params["Dynamixel.Params_" + s +".busId"]= aux;
		aux.value=list[2].toStdString();
		params["Dynamixel.Params_" + s +".invertedSign"]= aux;
		aux.value=list[3].toStdString();
		params["Dynamixel.Params_" + s +".minPos"]= aux;
		aux.value=list[4].toStdString();
		params["Dynamixel.Params_" + s +".maxPos"]= aux;
		aux.value=list[5].toStdString();
		params["Dynamixel.Params_" + s +".zeroPos"]= aux;
		aux.value=list[6].toStdString();	
		params["Dynamixel.Params_" + s +".maxVelocity"]= aux;
		aux.value=list[7].toStdString();	
		params["Dynamixel.Params_" + s +".stepsRange"]= aux;
		aux.value=list[8].toStdString();	
		params["Dynamixel.Params_" + s +".maxDegrees"]= aux;
	}
}

bool Monitor::sendParamsToWorker(RoboCompCommonBehavior::ParameterList config_params)
{
	if(checkParams(config_params) == true)
	{
		//Set params to worker
		if(worker->setParams(config_params) == true)
			return true;
	}
	rError("Incorrect parameters");
	return false;
}

//comprueba que los parametros sean correctos y los transforma a la estructura del worker
bool Monitor::checkParams(RoboCompCommonBehavior::ParameterList config_params)
{
	bool correct = true;
	if(QString::fromStdString(config_params["Dynamixel.NumMotors"].value).toFloat() < 1)
		correct = false;	

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
		}
	}
	std::cout << name << " " << value << std::endl;
	return true;
}
