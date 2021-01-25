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
	aux.editable = false;
	configGetString("NavigationAgent", "InnerModel", aux.value,"no file");
	params["InnerModel"] = aux;

	aux.editable = false;
	configGetString("NavigationAgent", "RobotName", aux.value,"robot");
	params["RobotName"] = aux;

	aux.editable = false;
	configGetString( "NavigationAgent","MaxZSpeed", aux.value,"600");
	params["MaxZSpeed"] = aux;

	aux.editable = false;
	configGetString( "NavigationAgent","MaxXSpeed", aux.value,"400");
	params["MaxXSpeed"] = aux;

	aux.editable = false;
	configGetString( "NavigationAgent","MaxRotationSpeed", aux.value,"0.9");
	params["MaxRotationSpeed"] = aux;

	aux.editable = false;
	configGetString( "NavigationAgent","RobotXWidth", aux.value,"500");
	params["RobotXWidth"] = aux;

	aux.editable = false;
	configGetString( "NavigationAgent","RobotZLong", aux.value,"500");
	params["RobotZLong"] = aux;

	aux.editable = false;
	configGetString( "NavigationAgent","RobotRadius", aux.value,"300");
	params["RobotRadius"] = aux;

	aux.editable = false;
	configGetString( "NavigationAgent","MinControllerPeriod", aux.value,"100");
	params["MinControllerPeriod"] = aux;

	aux.editable = false;
	configGetString( "NavigationAgent","PlannerGraphPoints", aux.value,"100");
	params["PlannerGraphPoints"] = aux;

	aux.editable = false;
	configGetString( "NavigationAgent","PlannerGraphNeighbours", aux.value,"20");
	params["PlannerGraphNeighbours"] = aux;

	aux.editable = false;
	configGetString( "NavigationAgent","PlannerGraphMaxDistanceToSearch", aux.value,"2500");
	params["PlannerGraphMaxDistanceToSearch"] = aux;

	aux.editable = false;
	configGetString( "NavigationAgent","OuterRegionLeft", aux.value,"0");
	params["OuterRegionLeft"] = aux;

	aux.editable = false;
	configGetString( "NavigationAgent","OuterRegionRight", aux.value,"6000");
	params["OuterRegionRight"] = aux;

	aux.editable = false;
	configGetString( "NavigationAgent","OuterRegionBottom", aux.value,"-4250");
	params["OuterRegionBottom"] = aux;

	aux.editable = false;
	configGetString( "NavigationAgent","OuterRegionTop", aux.value,"4250");
	params["OuterRegionTop"] = aux;

	aux.editable = false;
	configGetString( "NavigationAgent","ExcludedObjectsInCollisionCheck", aux.value,"floor_plane");
	params["ExcludedObjectsInCollisionCheck"] = aux;

	aux.editable = false;
	configGetString( "NavigationAgent","MinimumDetectableRotation", aux.value,"0.03");
	params["MinimumDetectableRotation"] = aux;

	aux.editable = false;
	configGetString( "NavigationAgent","MinimumDetectableTranslation", aux.value,"10");
	params["MinimumDetectableTranslation"] = aux;

	aux.editable = false;
	configGetString( "NavigationAgent","MinimumSafetyDistance", aux.value, "100");
	params["MinimumSafetyDistance"] = aux;
    
    configGetString( "NavigationAgent","World", aux.value, "");
	params["World"] = aux;
}

//Check parameters and transform them to worker structure
bool SpecificMonitor::checkParams(RoboCompCommonBehavior::ParameterList l)
{
	bool correct = true;
	return correct;
}

