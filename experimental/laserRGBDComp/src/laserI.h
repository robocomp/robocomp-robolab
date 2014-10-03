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
#ifndef LASERI_H
#define LASERI_H

// QT includes
#include <QtCore/QObject>

// Ice includes
#include <Ice/Ice.h>
#include <Laser.h>

#include <config.h>
#include "worker.h"

using namespace RoboCompLaser;

class LaserI : public QObject , public virtual RoboCompLaser::Laser
{
Q_OBJECT
public:
	LaserI( Worker *_worker, QObject *parent = 0 );
	~LaserI();

	TLaserData getLaserData( const Ice::Current &c= Ice::Current());
	TLaserData getLaserAndBStateData(RoboCompDifferentialRobot::TBaseState &bState, const Ice::Current &c= Ice::Current());
	LaserConfData getLaserConfData( const Ice::Current &c= Ice::Current());


	QMutex *mutex;
private:

	Worker *worker;
public slots:


};

#endif
