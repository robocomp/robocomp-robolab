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
#ifndef STABLEODOMETRY_H
#define STABLEODOMETRY_H

// QT includes
#include <QtCore/QObject>

// Ice includes
#include <Ice/Ice.h>
#include <StableOdometry.h>

#include <config.h>
#include "genericworker.h"

using namespace RoboCompStableOdometry;

class StableOdometryI : public QObject , public virtual RoboCompStableOdometry::StableOdometry
{
Q_OBJECT
public:
	StableOdometryI( GenericWorker *_worker, QObject *parent = 0 );
	~StableOdometryI();
	
	void newStableOdometry(const float  x, const float  z, const float  alpha, const Ice::Current&);

	QMutex *mutex;
private:

	GenericWorker *worker;
public slots:


};

#endif
