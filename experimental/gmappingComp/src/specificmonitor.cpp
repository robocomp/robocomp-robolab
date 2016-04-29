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
	configGetString("", "GMapping.maxUrange", aux.value,"20.0");
	params["GMapping.maxUrange"] = aux;
	configGetString("", "GMapping.maxrange", aux.value,"30.0");
	params["GMapping.maxrange"] = aux;
// 	configGetString("", "GMapping.sigma", aux.value,"0.1");
	configGetString("", "GMapping.sigma", aux.value,"0.05");
	params["GMapping.sigma"] = aux;
	configGetString("", "GMapping.regscore", aux.value,"10000.0");
	params["GMapping.regscore"] = aux;
	configGetString("", "GMapping.kernelSize", aux.value,"1");
	params["GMapping.kernelSize"] = aux;
	configGetString("", "GMapping.lstep", aux.value,"0.05");
	params["GMapping.lstep"] = aux;
	configGetString("", "GMapping.astep", aux.value,"0.05");
	params["GMapping.astep"] = aux;
	configGetString("", "GMapping.maxMove", aux.value,"1");
	params["GMapping.maxMove"] = aux;
	configGetString("", "GMapping.iterations", aux.value,"10");
// 	configGetString("", "GMapping.iterations", aux.value,"5");
	params["GMapping.iterations"] = aux;
	configGetString("", "GMapping.lsigma", aux.value,"0.075");
// 	configGetString("", "GMapping.lsigma", aux.value,"0.75");
	params["GMapping.lsigma"] = aux;
	configGetString("", "GMapping.ogain", aux.value,"3");
	params["GMapping.ogain"] = aux;
	configGetString("", "GMapping.lskip", aux.value,"0");
	params["GMapping.lskip"] = aux;
	configGetString("", "GMapping.srr", aux.value,"0.1");
// 	configGetString("", "GMapping.srr", aux.value,"0.01");
	params["GMapping.srr"] = aux;
	configGetString("", "GMapping.srt", aux.value,"0.01");
	params["GMapping.srt"] = aux;
// 	configGetString("", "GMapping.str", aux.value,"0.01");
	configGetString("", "GMapping.str", aux.value,"0.1");
	params["GMapping.str"] = aux;
	configGetString("", "GMapping.stt", aux.value,"0.01");
	params["GMapping.stt"] = aux;
	configGetString("", "GMapping.linearUpdate", aux.value,"0.2");//"0.3");
	params["GMapping.linearUpdate"] = aux;
	configGetString("", "GMapping.angularUpdate", aux.value,"0.1");//"0.1");
	params["GMapping.angularUpdate"] = aux;
	configGetString("", "GMapping.resampleThreshold", aux.value,"0.5");
	params["GMapping.resampleThreshold"] = aux;
	configGetString("", "GMapping.xmin", aux.value,"-20.0");
	params["GMapping.xmin"] = aux;
	configGetString("", "GMapping.xmax", aux.value,"20.0");
	params["GMapping.xmax"] = aux;
	configGetString("", "GMapping.ymin", aux.value,"-20.0");
	params["GMapping.ymin"] = aux;
	configGetString("", "GMapping.ymax", aux.value,"20.0");
	params["GMapping.ymax"] = aux;
	configGetString("", "GMapping.particles", aux.value,"50");
	params["GMapping.particles"] = aux;
	printf("PARTICLES: %s\n", aux.value.c_str());
	
	configGetString("", "GMapping.delta", aux.value,"0.05");
	params["GMapping.delta"] = aux;
	configGetString("", "GMapping.llsamplerange", aux.value,"0.1");
	params["GMapping.llsamplerange"] = aux;
	configGetString("", "GMapping.llsamplestep", aux.value,"0.1");
	params["GMapping.llsamplestep"] = aux;
	configGetString("", "GMapping.lasamplerange", aux.value,"0.01");
	params["GMapping.lasamplerange"] = aux;
	configGetString("", "GMapping.lasamplestep", aux.value,"0.02");
	params["GMapping.llsamplestep"] = aux;
	configGetString("", "GMapping.enlargestep", aux.value,"1");
	params["GMapping.enlargestep"] = aux;
	
	
	
	configGetString("", "GMapping.generateMap", aux.value, "true");
	params["GMapping.generateMap"] = aux;


	try{
		configGetString("", "GMapping.Map", aux.value, "");
	} catch(...)
	{
		aux.value = "";
	}
	params["GMapping.Map"] = aux;

	
	configGetString("", "GMapping.tx", aux.value,"0");
	params["GMapping.tx"] = aux;
	configGetString("", "GMapping.tz", aux.value,"0");
	params["GMapping.tz"] = aux;
	configGetString("", "GMapping.ry", aux.value,"0");
	params["GMapping.ry"] = aux;

	
}

//comprueba que los parametros sean correctos y los transforma a la estructura del worker
bool SpecificMonitor::checkParams(RoboCompCommonBehavior::ParameterList l)
{
	bool correct = true;
	return correct;
}

