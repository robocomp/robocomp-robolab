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
#ifndef GENERICWORKER_H
#define GENERICWORKER_H

#include "config.h"
#include <stdint.h>
#include <qlog/qlog.h>


#include <CommonBehavior.h>

#include <CameraSimple.h>
<<<<<<< HEAD
=======
#include <GetAprilTags.h>
>>>>>>> eea2392695743c234d5db54b3b0e434d726fbb1b

#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100

using namespace std;
using namespace RoboCompCameraSimple;
<<<<<<< HEAD

using TuplePrx = std::tuple<>;
=======
using namespace RoboCompGetAprilTags;

using TuplePrx = std::tuple<RoboCompGetAprilTags::GetAprilTagsPrxPtr>;
>>>>>>> eea2392695743c234d5db54b3b0e434d726fbb1b


class GenericWorker :
public QObject
{
Q_OBJECT
public:
	GenericWorker(TuplePrx tprx);
	virtual ~GenericWorker();
	virtual void killYourSelf();
	virtual void setPeriod(int p);

	virtual bool setParams(RoboCompCommonBehavior::ParameterList params) = 0;
	QMutex *mutex;


<<<<<<< HEAD
=======
	GetAprilTagsPrxPtr getapriltags_proxy;
>>>>>>> eea2392695743c234d5db54b3b0e434d726fbb1b

	virtual void CameraSimple_getImage(TImage &im) = 0;

protected:
	QTimer timer;
	int Period;

private:


public slots:
	virtual void compute() = 0;
	virtual void initialize(int period) = 0;
signals:
	void kill();
};

#endif
