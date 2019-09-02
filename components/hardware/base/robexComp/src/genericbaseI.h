/*
 *    Copyright (C)2019 by YOUR NAME HERE
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
#ifndef GENERICBASE_H
#define GENERICBASE_H

// Ice includes
#include <Ice/Ice.h>
#include <GenericBase.h>

#include <config.h>
#include "genericworker.h"

using namespace RoboCompGenericBase;

class GenericBaseI : public virtual RoboCompGenericBase::GenericBase
{
public:
	GenericBaseI(GenericWorker *_worker);
	~GenericBaseI();

	void getBaseState( RoboCompGenericBase::TBaseState  &state, const Ice::Current&);
	void getBasePose( int  &x,  int  &z,  float  &alpha, const Ice::Current&);

private:

	GenericWorker *worker;

};

#endif
