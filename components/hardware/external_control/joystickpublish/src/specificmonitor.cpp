
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
	ready=true;
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
	emit initializeWorker(period);
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
	//Read params from config file
	RoboCompCommonBehavior::Parameter aux;

    aux.editable = false;
    configGetString( "","joystickUniversal.robot_id", aux.value,"0");
    params["joystickUniversal.robot_id"] = aux;

	configGetString( "","joystickUniversal.Device", aux.value,"/dev/input/js0");
	params["joystickUniversal.Device"] = aux;
	
	aux.editable = false;
	configGetString( "","joystickUniversal.NumAxes", aux.value,"0");
	params["joystickUniversal.NumAxes"] = aux;
	
	aux.editable = false;
	configGetString( "","joystickUniversal.NumButtons", aux.value,"0");
	params["joystickUniversal.NumButtons"] = aux;
	
	aux.editable = false;
	configGetString( "","joystickUniversal.BasicPeriod", aux.value,"100");
	params["joystickUniversal.BasicPeriod"] = aux;
	
	aux.editable = false;
	configGetString( "","joystickUniversal.NormalizationValue", aux.value,"10");
	params["joystickUniversal.NormalizationValue"] = aux;
	
	aux.editable = false;
	configGetString( "","joystickUniversal.VelocityAxis", aux.value,"vel");
	params["joystickUniversal.VelocityAxis"] = aux;
	
	aux.editable = false;
	configGetString( "","joystickUniversal.DirectionAxis", aux.value,"dir");
	params["joystickUniversal.DirectionAxis"] = aux;
	
	for (int i=0; i < atoi(params.at("joystickUniversal.NumAxes").value.c_str()); i++)
	{
		aux.editable = false;
		std::string s = QString::number(i).toStdString();
		configGetString("", "joystickUniversal.Axis_" + s, aux.value , "4");
		params["joystickUniversal.Axis_" + s] = aux;
		rDebug("joystickUniversal.Axis_"+QString::fromStdString(s)+" = " + QString::fromStdString(params.at("joystickUniversal.Axis_" + s).value));
		QStringList list = QString::fromStdString(aux.value).split(",");

		if (list.size() != 6)
			qFatal("joystickUniversalComp::Monitor::readConfig(): ERROR reading axis. Only %d parameters allowed %d.", list.size(), i);
		
		aux.value=list[0].toStdString();
		params["joystickUniversal.Axis_" + s +".Name"] = aux;
		rDebug("joystickUniversal.Axis_" + s + ".Name = " + params.at("joystickUniversal.Axis_" + s +".Name").value);
        aux.value=list[1].toStdString();
        params["joystickUniversal.Axis_" + s +".Axis"] = aux;
        rDebug("joystickUniversal.Axis_" + s + ".Axis = " + params.at("joystickUniversal.Axis_" + s +".Axis").value);
        aux.value=list[2].toStdString();
		params["joystickUniversal.Axis_" + s +".MinRange"]= aux;
		rDebug("joystickUniversal.Axis_"+s+".MinRange = "+ params["joystickUniversal.Axis_" + s +".MinRange"].value);
		aux.value=list[3].toStdString();
		params["joystickUniversal.Axis_" + s +".MaxRange"]= aux;
		rDebug("joystickUniversal.Axis_"+s+".MaxRange = "+ params["joystickUniversal.Axis_" + s +".MaxRange"].value);
		aux.value=list[4].toStdString();
		params["joystickUniversal.Axis_" + s +".Inverted"]= aux;
		rDebug("joystickUniversal.Axis_"+s+".Inverted = "+ params["joystickUniversal.Axis_" + s +".Inverted"].value);
        aux.value=list[5].toStdString();
		params["joystickUniversal.Axis_" + s +".DeadZone"]= aux;
		rDebug("joystickUniversal.Axis_"+s+".DeadZone = "+ params["joystickUniversal.Axis_" + s +".DeadZone"].value);

	}
    qInfo() << __FUNCTION__ << "---------------------";
    for (int i=0; i < atoi(params.at("joystickUniversal.NumButtons").value.c_str()); i++)
    {
        aux.editable = false;
        std::string s = QString::number(i).toStdString();
        configGetString("", "joystickUniversal.Button_" + s, aux.value , "0");
        params["joystickUniversal.Button_" + s] = aux;
        rDebug("joystickUniversal.Button_"+QString::fromStdString(s)+" = " + QString::fromStdString(params.at("joystickUniversal.Button_" + s).value));
        QStringList list = QString::fromStdString(aux.value).split(",");
        if (list.size() != 3)
            qFatal("joystickUniversalComp::Monitor::readConfig(): ERROR reading axis. Only %d params allowed %d.", list.size(), i);

        aux.value=list[0].toStdString();
        params["joystickUniversal.Button_" + s +".Name"] = aux;
        rDebug("joystickUniversal.Button_" + s + ".Name = " + params.at("joystickUniversal.Button_" + s +".Name").value);
        aux.value=list[1].toStdString();
        params["joystickUniversal.Button_" + s +".Number"] = aux;
        rDebug("joystickUniversal.Button_" + s + ".Number = " + params.at("joystickUniversal.Button_" + s +".Number").value);
        aux.value=list[2].toStdString();
        params["joystickUniversal.Button_" + s +".Step"]= aux;
        rDebug("joystickUniversal.Button_"+s+".Step = "+ params["joystickUniversal.Button_" + s +".Step"].value);
    }
}

//comprueba que los parametros sean correctos y los transforma a la estructura del worker
bool SpecificMonitor::checkParams(RoboCompCommonBehavior::ParameterList l)
{
	bool correct = true;
	return correct;
}

