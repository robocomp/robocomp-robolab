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
	state = RoboCompCommonBehavior::State::Running;
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
	RoboCompCommonBehavior::Parameter aux;
	int num_cameras;
	aux.editable = false;



    configGetString( "","origen_rx", aux.value, "");
    params["origen_rx"] = aux;

    configGetString( "","origen_ry", aux.value, "");
    params["origen_ry"] = aux;

    configGetString( "","origen_rz", aux.value, "");
    params["origen_rz"] = aux;

    configGetString( "","origen_tx", aux.value, "");
    params["origen_tx"] = aux;

    configGetString( "","origen_ty", aux.value, "");
    params["origen_ty"] = aux;

    configGetString( "","origen_tz", aux.value, "");
    params["origen_tz"] = aux;

    configGetString( "","num_cameras", aux.value, "");
    params["num_cameras"] = aux;
    num_cameras = std::stoi(params["num_cameras"].value);
    
    configGetString( "","odometry", aux.value, "");
    params["odometry"] = aux;


    for(int i = 0; i<num_cameras; i++){

        configGetString( "","device_serial_" + std::to_string(i), aux.value, "");
        params["device_serial_"+std::to_string(i)] = aux;

        configGetString( "","name_"+std::to_string(i), aux.value, "");
        params["name_"+std::to_string(i)] = aux;

        configGetString( "","rx_"+std::to_string(i), aux.value, "");
        params["rx_"+std::to_string(i)] = aux;

        configGetString( "","ry_"+std::to_string(i), aux.value, "");
        params["ry_"+std::to_string(i)] = aux;

        configGetString( "","rz_"+std::to_string(i), aux.value, "");
        params["rz_"+std::to_string(i)] = aux;

        configGetString( "","tx_"+std::to_string(i), aux.value, "");
        params["tx_"+std::to_string(i)] = aux;

        configGetString( "","ty_"+std::to_string(i), aux.value, "");
        params["ty_"+std::to_string(i)] = aux;

        configGetString( "","tz_"+std::to_string(i), aux.value, "");
        params["tz_"+std::to_string(i)] = aux;
    }

	configGetString( "","print", aux.value, "false");
    params["print"] = aux;
}

//Check parameters and transform them to worker structure
bool SpecificMonitor::checkParams(RoboCompCommonBehavior::ParameterList l)
{
	bool correct = true;
	return correct;
}

