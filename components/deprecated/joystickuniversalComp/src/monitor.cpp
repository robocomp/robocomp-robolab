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
		rDebug("Sending params to worker");
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
	rDebug("------------------------------------");
	//Read params from config file
	//Example
	RoboCompCommonBehavior::Parameter aux;
	aux.editable = true;
	configGetString( "joystickUniversal.Device", aux.value,"/dev/ttyUSB0");
	params["joystickUniversal.Device"] = aux;
	rDebug("joystickUniversal.Device = "+ params["joystickUniversal.Device"].value);
	configGetString( "joystickUniversal.NumAxes", aux.value,"");
	params["joystickUniversal.NumAxes"] = aux;
	rDebug("joystickUniversal.NumAxes = "+ params["joystickUniversal.NumAxes"].value);
	int num_axes=0;
	num_axes = QString::fromStdString(aux.value).toInt();
	configGetString( "joystickUniversal.NumButtons", aux.value,"");
	params["joystickUniversal.NumButtons"] = aux;
	rDebug("joystickUniversal.NumButtons = "+ params["joystickUniversal.NumButtons"].value);
	int num_buttons=0;
	num_buttons = QString::fromStdString(aux.value).toInt();

	configGetString( "joystickUniversal.BasicPeriod", aux.value,"100");
	params["joystickUniversal.BasicPeriod"] = aux;
	rDebug("joystickUniversal.BasicPeriod = "+ params["joystickUniversal.BasicPeriod"].value);
	configGetString( "joystickUniversal.NormalizationValue", aux.value,"100");
	params["joystickUniversal.NormalizationValue"] = aux;
	rDebug("joystickUniversal.NormalizationValue = "+ params["joystickUniversal.NormalizationValue"].value);
	
	
	configGetString( "joystickUniversal.VelocityAxis", aux.value, "");
	params["joystickUniversal.VelocityAxis"] = aux;
	rDebug("joystickUniversal.VelocityAxis = "+ params["joystickUniversal.VelocityAxis"].value);
	
	configGetString( "joystickUniversal.DirectionAxis", aux.value, "");
	params["joystickUniversal.DirectionAxis"] = aux;
	rDebug("joystickUniversal.DirectionAxis = "+ params["joystickUniversal.DirectionAxis"].value);

	if(num_axes <= 0)
		qFatal("joystickUniversalComp::Monitor::readConfig(): ERROR No valid axes number.");
	std::string paramsStr;
	for (int i=0; i<num_axes; i++)
	{
		std::string s= QString::number(i).toStdString();
		configGetString( "joystickUniversal.Axis_" + s, aux.value , "");
		params["joystickUniversal.Axis_" + s] = aux;
		rDebug("joystickUniversal.Axis_"+QString::fromStdString(s)+" = "+QString::fromStdString(params["joystickUniversal.Axis_" + s].value));
		QStringList list = QString::fromStdString(aux.value).split(",");
		if (list.size() != 4)	
			qFatal("joystickUniversalComp::Monitor::readConfig(): ERROR reading axis. Only %d parameters for motor %d.", list.size(), i);
		
		aux.value=list[0].toStdString();
		params["joystickUniversal.Axis_" + s +".Name"]= aux;
		rDebug("joystickUniversal.Axis_"+s+".Name = "+ params["joystickUniversal.Axis_" + s +".Name"].value);
		aux.value=list[1].toStdString();
		params["joystickUniversal.Axis_" + s +".MinRange"]= aux;
		rDebug("joystickUniversal.Axis_"+s+".MinRange = "+ params["joystickUniversal.Axis_" + s +".MinRange"].value);
		aux.value=list[2].toStdString();
		params["joystickUniversal.Axis_" + s +".MaxRange"]= aux;
		rDebug("joystickUniversal.Axis_"+s+".MaxRange = "+ params["joystickUniversal.Axis_" + s +".MaxRange"].value);
		aux.value=list[3].toStdString();
		params["joystickUniversal.Axis_" + s +".Inverted"]= aux;
		rDebug("joystickUniversal.Axis_"+s+".Inverted = "+ params["joystickUniversal.Axis_" + s +".Inverted"].value);

	}
}

//comprueba que los parametros sean correctos y los transforma a la estructura del worker
bool Monitor::checkParams(RoboCompCommonBehavior::ParameterList l)
{
	bool correct = true;
	// COPROBAR QUE EXISTE LOS EJES DE DIRECCION Y VELOCIDAD CON LOS NOMBRES ESTABLECIDOS
	// COMPROBAR QUE EL NUMERO DE EJES ES MAYOR QUE 0
	
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
	return true;
}
