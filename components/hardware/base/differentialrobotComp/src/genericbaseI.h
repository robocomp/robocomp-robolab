/*
 *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
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
#ifndef GENERICBASEI_H
#define GENERICBASEI_H

// QT includes
#include <QtCore/QObject>

// Ice includes
#include <Ice/Ice.h>
#include <GenericBase.h>

#include <config.h>
#include "worker.h"

using namespace RoboCompGenericBase;

class GenericBaseI : public QObject , public virtual RoboCompGenericBase::GenericBase
{
Q_OBJECT
public:
	GenericBaseI( Worker *_worker, QObject *parent = 0 );
	~GenericBaseI();
	void getBaseState(RoboCompGenericBase::TBaseState& state, const Ice::Current& = Ice::Current());
	void getBasePose(Ice::Int& x, Ice::Int& z, Ice::Float& alpha, const Ice::Current& = Ice::Current());

	QMutex *mutex;
private:
	void throwException(std::string msg);
	Worker *worker;
};

#endif