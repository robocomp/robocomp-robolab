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


#ifndef FALCONHANDLER_H
#define FALCONHANDLER_H

#include <falcon/core/FalconLogger.h>
#include <falcon/core/FalconDevice.h>
#include <falcon/firmware/FalconFirmwareNovintSDK.h>
#include <falcon/grip/FalconGripFourButton.h>
#include <falcon/kinematic/FalconKinematicStamper.h>
#include <falcon/util/FalconFirmwareBinaryNvent.h>
#include <boost/shared_ptr.hpp>

#include <QThread>
#include <QVector3D>

const unsigned int FALCON_BUTTON_CENTER = libnifalcon::FalconGripFourButton::BUTTON_3;
const unsigned int FALCON_BUTTON_LEFT = libnifalcon::FalconGripFourButton::BUTTON_4;
const unsigned int FALCON_BUTTON_RIGHT = libnifalcon::FalconGripFourButton::BUTTON_1;
const unsigned int FALCON_BUTTON_TOP = libnifalcon::FalconGripFourButton::BUTTON_2;



class FalconHandler  : public QThread
{
	Q_OBJECT
	
private:
	std::shared_ptr<libnifalcon::FalconFirmware> firmware;
	libnifalcon::FalconDevice device;
	QVector3D normalizedPos;
	uint32_t buttons;
	
public:
    FalconHandler( const unsigned int numDev, QObject* parent = 0);
    ~FalconHandler();
	void run();
	QVector3D getPosition() const;
	uint32_t getButtons() const;
};

#endif // FALCONHANDLER_H
