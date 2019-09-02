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
#ifndef JOYSTICKI_H
#define JOYSTICKI_H

#include <string>
#include <iostream>

#include <QtCore/QObject>
#include <QList>
#include <QListIterator>

#include <IceUtil/UUID.h>

#include <JoyStick.h>

using namespace RoboCompJoyStick;

class JoyStickI : public QObject , public virtual RoboCompJoyStick::JoyStick
{
    Q_OBJECT
    public:
        JoyStickI(  QObject *parent = 0 );
        ~JoyStickI();
    	
        void readJoyStickBufferedData( RoboCompJoyStick::JoyStickBufferedData &gbd, const Ice::Current& = Ice::Current() ) ;
        void writeJoyStickBufferedData( const RoboCompJoyStick::JoyStickBufferedData &gbd, const Ice::Current& = Ice::Current() );
      // Component public functions

    private:
        RoboCompJoyStick::JoyStickBufferedData joystickBufferedData; 
    public slots:
        // Component public slots
};

#endif
