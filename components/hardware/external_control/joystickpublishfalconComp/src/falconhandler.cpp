/*
    <one line to give the library's name and an idea of what it does.>
    Copyright (C) 2011  <copyright holder> <email>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/


#include "falconhandler.h"

#include <stdio.h>

const unsigned int MAX_ATTEMPTS = 50;
const QVector3D scale( 16.503656, 16.336500, 19.758589 );
const QVector3D origin( -0.006103, 0.014811, -2.462171 );
const double stiffness = 3.0;



FalconHandler::FalconHandler(const unsigned int numDev, QObject* parent) : QThread(parent)
{
	// Try to open the Falcon device
	printf( "Opening Falcon #%d...\n", numDev+1 );
	if( !device.open( numDev ) ) {
		throw "Cannot open falcon.";
	}
	
	// Try to load the firmware
	device.setFalconFirmware< libnifalcon::FalconFirmwareNovintSDK >();
	if( !device.isFirmwareLoaded() ) {
		for( unsigned int i=0 ; i<MAX_ATTEMPTS; i++ ) {
			printf( "Loading firmware (attempt %d/%d)...\n", i+1, MAX_ATTEMPTS );
			const bool loaded = device.getFalconFirmware()->loadFirmware(
				true,
				libnifalcon::NOVINT_FALCON_NVENT_FIRMWARE_SIZE,
				const_cast<uint8_t*>( libnifalcon::NOVINT_FALCON_NVENT_FIRMWARE ) );
			if( loaded ) break;
		}
	}
	firmware = device.getFalconFirmware();
	
	// Check that the firmware has been correctly loaded
	if( device.isFirmwareLoaded() ) {
		printf( "Falcon initialized correctly.\n" );
	} else {
		throw "Couldn't load firmware.";
	}
	
	// Calibrate the kinematics
	device.setFalconKinematic< libnifalcon::FalconKinematicStamper >();
	device.setFalconGrip< libnifalcon::FalconGripFourButton >();
	printf( "Calibrating, move the end effector all the way out.\n" );
	forever {
		if( !device.runIOLoop() ) continue;
		if( device.getPosition()[2] > .170)  break;
	}
	printf( "Calibration done.\n" );
}



FalconHandler::~FalconHandler()
{
	device.close();
}



void FalconHandler::run()
{
    forever
	{
		// Update the position
		device.runIOLoop();
		std::array<double, 3> devPos = device.getPosition();
		normalizedPos = origin + scale*QVector3D( devPos[0], devPos[1], devPos[2] );
		buttons = device.getFalconGrip()->getDigitalInputs();
		
		// Compute the tension
		std::array<double, 3> force;
		force[0] = -stiffness * normalizedPos.x();
		force[1] = -stiffness * normalizedPos.y() + 0.98;
		force[2] = -stiffness * normalizedPos.z();
		device.setForce( force );
	}
}



QVector3D FalconHandler::getPosition() const
{
	return normalizedPos;
}



uint32_t FalconHandler::getButtons() const
{
	return buttons;
}
