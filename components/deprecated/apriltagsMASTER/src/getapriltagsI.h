/*
 *    Copyright (C) 2017 by YOUR NAME HERE
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
#ifndef GETAPRILTAGS_H
#define GETAPRILTAGS_H

// QT includes
#include <QtCore/QObject>

// Ice includes
#include <Ice/Ice.h>
#include <GetAprilTags.h>

#include <config.h>
#include "genericworker.h"

using namespace RoboCompGetAprilTags;

class GetAprilTagsI : public QObject , public virtual RoboCompGetAprilTags::GetAprilTags
{
Q_OBJECT
public:
	GetAprilTagsI( GenericWorker *_worker, QObject *parent = 0 );
	~GetAprilTagsI();
	
	listaMarcas checkMarcas(const Ice::Current&);

	QMutex *mutex;
private:

	GenericWorker *worker;
public slots:


};

#endif
