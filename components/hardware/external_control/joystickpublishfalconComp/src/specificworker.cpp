/*
 *    Copyright (C) 2015 by YOUR NAME HERE
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

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	falcon = new FalconHandler(0);
	falcon->start();
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	qDebug()<< "setParams";
	timer.start(50);
	return true;
}

void SpecificWorker::compute()
{	
	qDebug()<< "COMPUTE";
	RoboCompJoystickAdapter::TData data;
	RoboCompJoystickAdapter::AxisParams axisTemp;
	RoboCompJoystickAdapter::ButtonParams buttonTemp;
	
	// Get the position of the axes
	const QVector3D position = falcon->getPosition();
	axisTemp.name = "x";
	axisTemp.value = position.x();
	data.axes.push_back( axisTemp );
	
	axisTemp.name = "y";
	axisTemp.value = position.y();
	data.axes.push_back( axisTemp );
	
	axisTemp.name = "z";
	axisTemp.value = -position.z();
	data.axes.push_back( axisTemp );
	
	// Get the state of the buttons
	const uint32_t buttons = falcon->getButtons();
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
		joystickadapter_proxy->sendData( data );
		
		qDebug() << "Axis";
		for( auto a : data.axes)
		{
			std::cout << "	" << a.name << " " << a.value << std::endl;
		}
		qDebug() << "Buttons";
		for( auto b : data.buttons)
		{
			std::cout << "	" << b.clicked << std::endl;
		}
		qDebug() << "--------------------------";
	}
	catch( const Ice::Exception& e ) 
	{
		std::cout << e << " in Compute" << std::endl;
	}
}





