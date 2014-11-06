/*
 *    Copyright (C) 2006-2011 by RoboLab - University of Extremadura
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
#include "worker.h"



/**
* \brief Default constructor
*/
Worker::Worker( unsigned int numDev, RoboCompJoystickAdapter::JoystickAdapterPrx joystickAdapterPrx,RoboCompJoystickAdapter::JoystickAdapterPrx joystickAdapterPrx2 ) : falcon(numDev)
{
	// Initialize the attributes
	joystickAdapter = joystickAdapterPrx;
	joystickAdapter2 = joystickAdapterPrx2;
	Period = BASIC_PERIOD*1000;
	mutex = new QMutex();
	falcon.start();
}



/**
* \brief Default destructor
*/
Worker::~Worker()
{

}
///Common Behavior



void Worker::killYourSelf()
{
	rDebug("Killing myself");
	emit kill();
	exit(1);
}



/**
* \brief Change compute period
* @param per Period in ms
*/
void Worker::setPeriod(int p)
{
	rDebug("Period changed"+QString::number(p));
	Period = p*1000;
}



/**
* \brief
* @param params_ Parameter list received from monitor thread
*/
bool Worker::setParams(RoboCompCommonBehavior::ParameterList params_)
{
//	active = false;
		//CAMBIAR PARAMETROS Y RE-ARRANQUE DEL COMPONENTE SI ES NECESARIO

//	active = true;
	this->start();
	return true;
}



/**
* \brief Thread method
*/
void Worker::run()
{
	forever
	{
		RoboCompJoystickAdapter::TData data;
		RoboCompJoystickAdapter::AxisParams axisTemp;
		RoboCompJoystickAdapter::ButtonParams buttonTemp;
		
		// Get the position of the axes
		const QVector3D position = falcon.getPosition();
		axisTemp.name = "x";
		axisTemp.value = position.x();
		data.axes.push_back( axisTemp );
		
		axisTemp.name = "y";
		axisTemp.value = position.y();
		data.axes.push_back( axisTemp );
		
		axisTemp.name = "z";
		axisTemp.value = position.z();
		data.axes.push_back( axisTemp );
		
		// Get the state of the buttons
		const uint32_t buttons = falcon.getButtons();
		buttonTemp.clicked = buttons & FALCON_BUTTON_CENTER;
		data.buttons.push_back( buttonTemp );
		
		buttonTemp.clicked = buttons & FALCON_BUTTON_LEFT;
		data.buttons.push_back( buttonTemp );
		
		buttonTemp.clicked = buttons & FALCON_BUTTON_RIGHT;
		data.buttons.push_back( buttonTemp );
		
		buttonTemp.clicked = buttons & FALCON_BUTTON_TOP;
		data.buttons.push_back( buttonTemp );
		
		// Send the data
		data.velAxisIndex = 0;
		data.dirAxisIndex = 0;		
		try 
		{
			joystickAdapter->sendData( data );
			//joystickAdapter2->sendData( data );
		}
		catch( const Ice::Exception& e ) 
		{
			qWarning((QString("Error talking to joystickAdapter") + QString(e.what())).toStdString().c_str());
		}
		
		usleep(Period);
	}
}
