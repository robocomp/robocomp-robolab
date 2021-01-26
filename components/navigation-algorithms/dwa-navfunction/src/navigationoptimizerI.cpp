/*
 *    Copyright (C) 2021 by YOUR NAME HERE
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
#include "navigationoptimizerI.h"

NavigationOptimizerI::NavigationOptimizerI(GenericWorker *_worker)
{
	worker = _worker;
}


NavigationOptimizerI::~NavigationOptimizerI()
{
}


void NavigationOptimizerI::abort(const Ice::Current&)
{
	worker->NavigationOptimizer_abort();
}

RoboCompNavigationOptimizer::Params NavigationOptimizerI::getParams(const Ice::Current&)
{
	return worker->NavigationOptimizer_getParams();
}

RoboCompNavigationOptimizer::State NavigationOptimizerI::getState(const Ice::Current&)
{
	return worker->NavigationOptimizer_getState();
}

bool NavigationOptimizerI::gotoNewRandomPoint(RoboCompNavigationOptimizer::Params params, const Ice::Current&)
{
	return worker->NavigationOptimizer_gotoNewRandomPoint(params);
}

