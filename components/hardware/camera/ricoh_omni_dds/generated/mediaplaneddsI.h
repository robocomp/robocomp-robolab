/*
 *    Copyright (C) 2026 by YOUR NAME HERE
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
#ifndef MEDIAPLANEDDS_H
#define MEDIAPLANEDDS_H

// Ice includes
#include <Ice/Ice.h>
#include <MediaPlaneDDS.h>
#include <QPointer>

#include "../src/specificworker.h"


class MediaPlaneDDSI : public virtual RoboCompMediaPlaneDDS::MediaPlaneDDS
{
public:
	MediaPlaneDDSI(GenericWorker *_worker, const size_t id);
	~MediaPlaneDDSI();

	std::string getMediaDescriptor(const Ice::Current&);

private:

	QPointer<GenericWorker> worker;
	size_t id;

	// Array handlers for each method
	std::array<std::function<std::string(void)>, 1> getMediaDescriptorHandlers;

};

#endif
