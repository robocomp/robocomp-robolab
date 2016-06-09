/*
 *    Copyright (C) 2016 by YOUR NAME HERE
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

/**
       \brief
       @author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <qlog/qlog.h>
#include <rcdraw/rcdrawrobot.h>
#include <innermodel/innermodel.h>

#include <values.h>
#include <gridfastslam/gridslamprocessor.h>
#include "GridFastSlamMapHandling.h"

#define WIDGETWIDTH 600

using namespace GMapping;

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	bool saveMap(const string &path);
	void getWholeGrid(GridMap &map, Pose2D &pose);
	void initializeRobotPose(const Pose2D &pose);
	void getPartialGrid(const MapRect &rect, GridMap &map, Pose2D &pose);

	void drawMap(Map<double, DoubleArray2D, false>* mymap);
	void drawAllParticles(Map<double, DoubleArray2D, false>*);
	void drawBestParticle(Map<double, DoubleArray2D, false>*);
	void drawOdometry(Map<double, DoubleArray2D, false>*);
	void queueLastPose();
	void drawSquare(const QRect &rect);
	
public slots:
	void compute();
	void saveMap();
	void loadMap();
	void resetMap();
	void newWorldCoor(QPointF p);
	void regenerateRT();
	
	void iniMouseCoor(QPoint p)
	{
		pressEvent =         QVec::vec3(p.x(), 0, -p.y());
	}
	
	void endMouseCoor(QPoint p)
	{
		auto f = [](QVec &vc)
		{
			vc(0) -= WIDGETWIDTH/2;
			vc(0) *= 20000./600.;
			vc(2) += WIDGETWIDTH/2;
			vc(2) *= 20000./600.;
		};

		QVec releaseEvent = QVec::vec3(p.x(), 0, -p.y());
		f(releaseEvent);
		f(pressEvent);
		QVec inc = releaseEvent-pressEvent;
		pressEvent.print("pressEvent");
		releaseEvent.print("releaseEvent");
		inc.print("inc");
		printf("norm inc %f\n", inc.norm2());
		QVec zero = pressEvent;
		float r = -atan2(inc(2), inc(0));
		printf("calibration: %f %f __ %f\n", zero(0), zero(2), r);
	}
private:
	QVec pressEvent;
	
	
	RoboCompLaser::TLaserData laserData;
	RoboCompOmniRobot::TBaseState bState;
	float previousAlpha;

	bool active;
	RoboCompSlamLaser::GridMap interfaceMapA, interfaceMapB, *interfaceMapWrite, *interfaceMapRead;
	RoboCompSlamLaser::Pose2D interfacePoseA, interfacePoseB, *interfacePoseWrite, *interfacePoseRead;

	QMutex *mutex;
	QRect worldSize;
	int period;
	InnerModel *innerModel;
	RangeSensor *rs;
	SensorMap sensorMap;
	GridSlamProcessor* processor;
	bool processed;
	unsigned int best_idx;
	RoboCompCommonBehavior::ParameterList params;
	double xmin, xmax, ymin, ymax;
	int nParticles;
	bool registerScan;
	GMapping::RangeReading robocompWrapper(RoboCompOmniRobot::TBaseState usedState);
	double *distances_laser;
	void initialize();

	void updateMap(Map<double, DoubleArray2D, false>* mymap);
	uint8_t *gridMapBuffer;
	QVector<double> v2DData;

	QVec finalCorrection;
	RoboCompOmniRobot::TBaseState correction;
	RTMat mapTransform;
	float mapTransform_ry;
	
	RCDraw *map;
//	QGraphicsView *view;
//	QGraphicsScene *scene;
	QImage *result;
	QImage *map_buffer;
	QList<QVector <float>*> trajectory;
	
	
};

#endif

