/*
 *    Copyright (C) 2021 by YOUR NAME HERE
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
#include "specificworker.h"
#include "giraff_funciones.h"
#include "giraff_serial.h"
#include <time.h>
#include "BatteryStatus.h"

time_t time_actual;

BatteryState Bateria;
GiraffState Estado,EstadoActualizado;
Odometria DatosOdometria;
Botones DatosBotones;



/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
}
//
/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content



	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;

	if(!iniciarAvr){
		std::cout<<"ERROR:INICIO AVR"<<std::endl;
	}

	if(iniciarAvr())
		std::cout<<"Inicio correcto del AVR"<<std::endl;
	
	//OBTENCIÓN DATOS BATERÍA

	if(!getGiraffBatteryData(Bateria)){
		std::cout<<"Error en lectura de bateria"<<std::endl;
	}

	DatosOdometria.alpha=0;
	DatosOdometria.x=0;
	DatosOdometria.z=0;
	DatosOdometria.v=0;
	DatosOdometria.vg=0;

	DatosBotones.Dial=0;
	//Poner pantalla recta
	usleep(3000000);
	EstadoActualizado.tilt=0.7;

	setTilt(EstadoActualizado.tilt);

	EstadoActualizado=Estado;
	
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		timer.start(Period);
	}
}

void SpecificWorker::compute()
{
	//Actualización velocidad

	mutex->lock();
		
		if(!(EstadoActualizado.v == 0 and Estado.v==0) or !(EstadoActualizado.vg == 0 and Estado.vg==0))

		{
			//if (abs(Estado.vg) > 0.10 or abs(Estado.v) > 0.001 ){

				SetSpeedBase(Estado.v, Estado.vg);
				std::cout << "velocidad: " << Estado.v <<std::endl;
				std::cout << "velocidad_giro: " << Estado.vg <<std::endl;

			// }		
			EstadoActualizado = Estado;		
		}
		if(Estado.tilt!=EstadoActualizado.tilt)
		{
			std::cout << " Inclinación pantalla " << Estado.tilt << endl;
			EstadoActualizado.tilt=Estado.tilt;
			setTilt(EstadoActualizado.tilt);
			EstadoActualizado = Estado;	
		}
		
	mutex->unlock();
	getGiraffOdometria(DatosOdometria);
	get_button_data(DatosBotones);
	////Comprobación bateria
	time(&time_actual);
	double seconds = difftime(time_actual,Bateria.timeStamp);
	if (seconds>=10 && EstadoActualizado.v==0 && EstadoActualizado.ang_speed==0)
	{
		getGiraffBatteryData(Bateria);
	}

}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}


RoboCompBatteryStatus::TBattery SpecificWorker::BatteryStatus_getBatteryState()
{
	RoboCompBatteryStatus::TBattery bat;

	bat.percentage=Bateria.percentage;
	bat.voltage=Bateria.voltage;
	bat.current=Bateria.current;


	switch(Bateria.power_supply_status) 
	{
		case 1: 
		bat.state=RoboCompBatteryStatus::BatteryStates::Charged;  
		break;
		case 2: 
		bat.state=RoboCompBatteryStatus::BatteryStates::Charging;  
		break;
		case 3: 
		bat.state=RoboCompBatteryStatus::BatteryStates::Disconnected;  
		break;
		default: 
		bat.state=RoboCompBatteryStatus::BatteryStates::Disconnected;  
	}
	return bat;

}


void SpecificWorker::DifferentialRobot_correctOdometer(int x, int z, float alpha)
{
	
}

void SpecificWorker::DifferentialRobot_getBasePose(int &x, int &z, float &alpha)
{	

}	

void SpecificWorker::DifferentialRobot_getBaseState(RoboCompGenericBase::TBaseState &state)
{
	state.x=DatosOdometria.x;
	state.z=DatosOdometria.z;
	state.alpha=DatosOdometria.alpha;
	state.rotV=DatosOdometria.vg;
	if (EstadoActualizado.v<0){
		state.advVx=-DatosOdometria.v*1000;
		state.advVz=-DatosOdometria.v*1000;
	}
	else{
		state.advVz=DatosOdometria.v*1000;
		state.advVx=DatosOdometria.v*1000;
		
	}
}

void SpecificWorker::DifferentialRobot_resetOdometer()
{

}

void SpecificWorker::DifferentialRobot_setOdometer(RoboCompGenericBase::TBaseState state)
{
//implementCODE

}

void SpecificWorker::DifferentialRobot_setOdometerPose(int x, int z, float alpha)
{
	//QMutexLocker locker(mutex);
	//DatosOdometria.x=x;
	//DatosOdometria.z=z;
	//DatosOdometria.alpha=alpha;
}

void SpecificWorker::DifferentialRobot_setSpeedBase(float adv, float rot)
{
	QMutexLocker locker(mutex);
	Estado.v = adv/1000;
	Estado.vg = rot;

}

void SpecificWorker::DifferentialRobot_stopBase()
{
	QMutexLocker locker(mutex);
	Estado.v=0;
	Estado.vg=0;
}

RoboCompGiraffButton::Botones SpecificWorker::GiraffButton_getBotonesState()
{
	RoboCompGiraffButton::Botones Botones;

	QMutexLocker locker(mutex);
	Botones.Rojo=DatosBotones.Rojo;
	Botones.Verde=DatosBotones.Verde;
	Botones.Dial=DatosBotones.Dial;

}


RoboCompJointMotorSimple::MotorParams SpecificWorker::JointMotorSimple_getMotorParams(std::string motor)
{
//implementCODE

}

RoboCompJointMotorSimple::MotorState SpecificWorker::JointMotorSimple_getMotorState(std::string motor)
{
//implementCODE
RoboCompJointMotorSimple::MotorState motorScreen;
motorScreen.pos = EstadoActualizado.tilt;
return motorScreen;

}

void SpecificWorker::JointMotorSimple_setPosition(std::string name, RoboCompJointMotorSimple::MotorGoalPosition goal)
{
//implementCODE
QMutexLocker locker(mutex);
	Estado.tilt=goal.position;
}

void SpecificWorker::JointMotorSimple_setVelocity(std::string name, RoboCompJointMotorSimple::MotorGoalVelocity goal)
{
//implementCODE

}

void SpecificWorker::JointMotorSimple_setZeroPos(std::string name)
{
//implementCODE

}


/**************************************/
// From the RoboCompBatteryStatus you can use this types:
// RoboCompBatteryStatus::TBattery

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompGiraffButton you can use this types:
// RoboCompGiraffButton::Botones

/**************************************/
// From the RoboCompJointMotorSimple you can use this types:
// RoboCompJointMotorSimple::MotorState
// RoboCompJointMotorSimple::MotorParams
// RoboCompJointMotorSimple::MotorGoalPosition
// RoboCompJointMotorSimple::MotorGoalVelocity

