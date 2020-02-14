/*
 *    Copyright (C) 2019 by YOUR NAME HERE
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
#include "genericbaseI.h"

GenericBaseI::GenericBaseI(GenericWorker *_worker)
{
	worker = _worker;
}


GenericBaseI::~GenericBaseI()
{
}

void GenericBaseI::getBaseState( RoboCompGenericBase::TBaseState  &state, const Ice::Current&)
{
	worker->GenericBase_getBaseState(state);
}

void GenericBaseI::getBasePose( int  &x,  int  &z,  float  &alpha, const Ice::Current&)
{
	worker->GenericBase_getBasePose(x, z, alpha);
}

