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

Monitor::Monitor( Worker *_worker, Ice::CommunicatorPtr _communicator )
{
	qDebug()<< "JointMotor::Monitor()";
	worker = _worker;
	this->communicator = _communicator;
	initializedOk= false;
	initialize();
}

Monitor::~Monitor()
{

}

void Monitor::run()
{
	forever
	{
		qDebug()<<"JointMotor::Monitor::run()";
		this->sleep(15);
	}
}

int Monitor::getFrequency()
{
	return worker->getFrequency(); // should we store a cached copy?
}

void Monitor::setFrequency(int freq)
{
	worker->setFrequency(freq);
}

void Monitor::killYourSelf()
{
	qDebug()<<"kill";
	this->exit();
	worker->exit();
	emit kill();
	
}
int Monitor::timeAwake()
{
	std::cout << "Setting freq..." << std::endl;
	return initialTime.secsTo(QTime::currentTime());
}


RoboCompCommonBehavior::ParameterList Monitor::getParameterList() 
{ 
	return params;
}
void Monitor::setParameterList(RoboCompCommonBehavior::ParameterList l) 
{ 
	sendParamsToWorker(l);
	qDebug()<<"set parameters";
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
  
//    Ice::PropertiesPtr props = communicator->getProperties();
//    std::string name = props->getProperty("Ice.Warn.Connections");
 	
	///Local Component parameters read at start

	std::cout << "JointMotorComp::Monitor::initialize()..." << std::endl;
	initialTime=QTime::currentTime();
	readConfig(params);
	if(!sendParamsToWorker(params))
	{
		std::cout << "Error reading config parameters. Exiting"<<std::endl;
		killYourSelf();
	}
	  
  return true;
}

///Local Component parameters read at start
///Reading parameters from config file or passed in command line, with Ice machinery
///We need to supply a list of accepted values to each call
void Monitor::readConfig(RoboCompCommonBehavior::ParameterList &params )
{
	qDebug()<<"Dunkermotoren::Monitor::readConfig()";
	RoboCompCommonBehavior::Parameter aux;
	aux.editable = true;
	//~ configGetString( "DRobot.Device", aux.value,"/dev/ttyUSB0");
	//~ params["DRobot.Device"] = aux;
	//~ configGetString( "DRobot.Handler", aux.value, "Robex" );  
	//~ params["DRobot.Handler"] = aux;
	//~ configGetString( "DRobot.maxVelAdv", aux.value, "150.0");  
	//~ params["DRobot.maxVelAdv"] = aux;
	//~ configGetString( "DRobot.maxVelRot", aux.value, "1.0");  
	//~ params["DRobot.maxVelRot"] = aux;

	configGetString( "dunkermotoren.NumMotors", aux.value, "" );	
	int num_motors=0;
	num_motors = QString::fromStdString(aux.value).toInt();
	if(num_motors <= 0) 
	  qFatal("ERROR: Dunkermotoren::Monitor::readConfig(): - Zero motors found. Exiting..." );

	params["dunkermotoren.NumMotors"] = aux;
	
	configGetString( "dunkermotoren.Device", aux.value, "" );	
	params["dunkermotoren.Device"] = aux;
	
	configGetString( "dunkermotoren.BaudRate", aux.value, "" );
	params["dunkermotoren.BaudRate"] = aux;
	
	configGetString( "dunkermotoren.BasicPeriod", aux.value, "" );
	params["dunkermotoren.BasicPeriod"] = aux;
	
	//~ configGetInt( "JointMotor.BaudRate", busParams->baudRate, 115200);  
	//~ configGetInt( "JointMotor.BasicPeriod", busParams->basicPeriod, 100);  
	//~ 
	std::string paramsStr;
	for (int i=0; i<num_motors; i++)
	{
		std::string s= QString::number(i).toStdString();
		configGetString( "dunkermotoren.Params_" + s, aux.value , "");
		params["dunkermotoren.Params_" + s] = aux;
		qDebug() << QString::fromStdString(aux.value);
		QStringList list = QString::fromStdString(aux.value).split(",");
		if (list.size() != 26) qFatal("ERROR Dunkermotoren::Monitor::readConfig(): error reading motor. Only %d (of 26) parameters for motor %d.", list.size(), i);
		
		aux.value=list[0].toStdString();
		params["dunkermotoren.Params_" + s +".name"]= aux;
		aux.value=list[1].toStdString();
		params["dunkermotoren.Params_" + s +".busId"]= aux;
		aux.value=list[2].toStdString();
		params["dunkermotoren.Params_" + s +".invertedSign"]= aux;
		aux.value=list[3].toStdString();
		params["dunkermotoren.Params_" + s +".minPos"]= aux;
		aux.value=list[4].toStdString();
		params["dunkermotoren.Params_" + s +".maxPos"]= aux;
		aux.value=list[5].toStdString();
		params["dunkermotoren.Params_" + s +".zeroPos"]= aux;
		aux.value=list[6].toStdString();	
		params["dunkermotoren.Params_" + s +".maxVelocity"]= aux;
		aux.value=list[7].toStdString();	
		params["dunkermotoren.Params_" + s +".stepsToRadsRatio"]= aux;
		aux.value=list[8].toStdString();	
		params["dunkermotoren.Params_" + s +".zeroPosRads"]= aux;
		aux.value=list[9].toStdString();	
		params["dunkermotoren.Params_" + s +".maxPosErr"]= aux;
		aux.value=list[10].toStdString();	
		params["dunkermotoren.Params_" + s +".accV"]= aux;
		aux.value=list[11].toStdString();	
		params["dunkermotoren.Params_" + s +".accT"]= aux;
		aux.value=list[12].toStdString();	
		params["dunkermotoren.Params_" + s +".decV"]= aux;
		aux.value=list[13].toStdString();	
		params["dunkermotoren.Params_" + s +".decT"]= aux;
		aux.value=list[14].toStdString();	
		params["dunkermotoren.Params_" + s +".gearRev"]= aux;
		aux.value=list[15].toStdString();	
		params["dunkermotoren.Params_" + s +".motorRev"]= aux;
		aux.value=list[16].toStdString();	
		params["dunkermotoren.Params_" + s +".posCurLim"]= aux;
		aux.value=list[17].toStdString();	
		params["dunkermotoren.Params_" + s +".negCurLim"]= aux;
		aux.value=list[18].toStdString();	
		params["dunkermotoren.Params_" + s +".curPeak"]= aux;
		aux.value=list[19].toStdString();	
		params["dunkermotoren.Params_" + s +".curContin"]= aux;
		aux.value=list[20].toStdString();	
		params["dunkermotoren.Params_" + s +".curTime"]= aux;
		aux.value=list[21].toStdString();	
		params["dunkermotoren.Params_" + s +".encodeRes"]= aux;
		aux.value=list[22].toStdString();	
		params["dunkermotoren.Params_" + s +".setPID"]= aux;
		aux.value=list[23].toStdString();	
		params["dunkermotoren.Params_" + s +".velKp"]= aux;
		aux.value=list[24].toStdString();	
		params["dunkermotoren.Params_" + s +".velKi"]= aux;
		aux.value=list[25].toStdString();	
		params["dunkermotoren.Params_" + s +".velKd"]= aux;
	}
}

bool Monitor::sendParamsToWorker(RoboCompCommonBehavior::ParameterList config_params)
{
	if (checkParams(config_params))
	{
		worker->setParams(config_params);
		return true;
	}
	else
	{
		qWarning("Incorrect parameters");
		return false;
	}
}

//comprueba que los parametros sean correctos y los transforma a la estructura del worker
bool Monitor::checkParams(RoboCompCommonBehavior::ParameterList config_params)
{
	bool correct = true;
	if(QString::fromStdString(config_params["dunkermotoren.NumMotors"].value).toFloat() < 1)
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
