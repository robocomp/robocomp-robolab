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

#define VISUALMAPWITDH 12000.
#define WIDGETWIDTH 700.

using namespace GMapping;

class SpecificWorker : public GenericWorker
{
Q_OBJECT

public:
	SpecificWorker(MapPrx &mprx);

	~SpecificWorker();

	bool setParams(RoboCompCommonBehavior::ParameterList params);

	RoboCompCommonBehavior::ParameterList getWorkerParams();

	bool saveMap(const string &path);

	void getWholeGrid(GridMap &map, Pose2D &pose);

	void initializeRobotPose(const Pose2D &pose);

	void getPartialGrid(const MapRect &rect, GridMap &map, Pose2D &pose);

	void drawMap(Map<double, DoubleArray2D, false> *mymap);

	void drawAllParticles(Map<double, DoubleArray2D, false> *);

	void drawAllLines();

	void drawBestParticle(Map<double, DoubleArray2D, false> *);

	void drawOdometry(Map<double, DoubleArray2D, false> *);

	void queueLastPose();

	void drawSquare(const QRect &rect);

public slots:

	void compute();

	void saveMap();

	void loadMap();

	void resetMap();

	// 	void newWorldCoor(QPointF p);
	void regenerateRT();

	void iniMouseCoor(QPoint p)
	{
		// 		printf("iniMouseCoor\n");
		pressEvent = QVec::vec3(p.x(), 0, -p.y());
		pressEvent.print("ini1");
		pressEvent = QVec::vec3(p.x() - float(WIDGETWIDTH) / 2., 0, -p.y() + float(WIDGETWIDTH) / 2.);
		pressEvent.print("ini2");
		auto f = [](QVec &vc) {
			vc(0) *= float(VISUALMAPWITDH) / float(WIDGETWIDTH);
			vc(2) *= float(VISUALMAPWITDH) / float(WIDGETWIDTH);
		};
		f(pressEvent);
		pressEvent.print("ini");
	}

	void endMouseCoor(QPoint p)
	{
		// 		printf("endMouseCoor\n");
		auto f = [](QVec &vc) {
			vc(0) -= float(WIDGETWIDTH) / 2;
			vc(0) *= VISUALMAPWITDH / float(WIDGETWIDTH);
			vc(2) += float(WIDGETWIDTH) / 2;
			vc(2) *= VISUALMAPWITDH / float(WIDGETWIDTH);
		};
		QVec releaseEvent = QVec::vec3(p.x(), 0, -p.y());
		f(releaseEvent);
		// 		releaseEvent.print("end");
		QVec inc = releaseEvent - pressEvent;
		// 		inc.print("inc");
		// 		printf("norm inc %f\n", inc.norm2());
		float r = atan2(inc(0), inc(2));


		if (action_cb->currentIndex() == 1)
		{
			std::pair<std::pair<float, float>, std::pair<float, float>> lineToDraw;
			lineToDraw.first.first = pressEvent(0);
			lineToDraw.first.second = pressEvent(2);
			lineToDraw.second.first = releaseEvent(0);
			lineToDraw.second.second = releaseEvent(2);
			linesToDraw.push_back(lineToDraw);
			QVec p1 = (mapTransform * pressEvent.toHomogeneousCoordinates()).fromHomogeneousCoordinates();
			QVec p2 = (mapTransform * releaseEvent.toHomogeneousCoordinates()).fromHomogeneousCoordinates();
			printf("m  %f,%f,%f,%f\n", p1(0) / 1000., p1(2) / 1000., p2(0) / 1000., p2(2) / 1000.);
			printf("mm %f,%f,%f,%f\n", p1(0), p1(2), p2(0), p2(2));
		}
		else if (action_cb->currentIndex() == 2)
		{
			printf(" - - - - - - - - -   PERFORMING RESET   - - - - - - - - - \n");
			int numParticles = QString::fromStdString(params["GMapping.particles"].value).toInt();
			//OrientedPoint OdomPose(p.y()/1000.f, p.x()/1000.f, 0.);
			std::vector<OrientedPoint> initialPose;
			QVec xg = QVec::uniformVector(numParticles, (pressEvent(2) / 1000.) - 0.5, (pressEvent(2) / 1000.) + 0.5);
			QVec yg = QVec::uniformVector(numParticles, (pressEvent(0) / 1000.) - 0.5, (pressEvent(0) / 1000.) + 0.5);
			QVec ag = QVec::uniformVector(numParticles, r - 0.9, r + 0.9);

			for (int i = 0; i < numParticles; i++)
			{
				initialPose.push_back(OrientedPoint(xg[i], yg[i], ag[i]));
			}

			if (params["GMapping.Map"].value.size() > 0)
			{
				ScanMatcherMap *loadedMap = GridFastSlamMapHandling::loadMap(params["GMapping.Map"].value);
				printf("processor->init(%d, %g, %g, %g, %g, %g, POSES)\n",
				       QString::fromStdString(params["GMapping.particles"].value).toInt(), xmin, ymin, xmax, ymax,
				       QString::fromStdString(params["GMapping.delta"].value).toDouble());

				processor->init(QString::fromStdString(params["GMapping.particles"].value).toInt(), xmin, ymin, xmax,
				                ymax, QString::fromStdString(params["GMapping.delta"].value).toDouble(), initialPose,
				                *loadedMap);
				delete loadedMap;
			}
			action_cb->setCurrentIndex(0);
		}
		else if (action_cb->currentIndex() == 3)
		{
			QVec zero = pressEvent;
			float r = -atan2(inc(2), inc(0));
			printf("calibration: %f %f __ %f\n", zero(0), zero(2), r);
			txSB->setValue(zero(0));
			tzSB->setValue(zero(2));
			rySB->setValue(r);
			regenerateRT();

		}
		else
		{
			printf("No action selected, check combo box options\n");
		}
	}

private:
	std::vector<std::pair<std::pair<float, float>, std::pair<float, float> > > linesToDraw;

	QVec pressEvent;


	RoboCompLaser::TLaserData laserData;
	RoboCompGenericBase::TBaseState bState;
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
	GridSlamProcessor *processor;
	bool processed;
	unsigned int best_idx;
	RoboCompCommonBehavior::ParameterList params;
	double xmin, xmax, ymin, ymax;
	int nParticles;
	bool registerScan;

	GMapping::RangeReading robocompWrapper(RoboCompGenericBase::TBaseState usedState);

	double *distances_laser;

	void initialize();

	void updateMap(Map<double, DoubleArray2D, false> *mymap);

	uint8_t *gridMapBuffer;
	QVector<double> v2DData;

	QVec finalCorrection;
// 	RoboCompGenericBase::TBaseState correction;
	RTMat mapTransform;
	float mapTransform_ry;

	RCDraw *map;
//	QGraphicsView *view;
//	QGraphicsScene *scene;
	QImage *result;
	QImage *map_buffer;
	QList<QVector<float> *> trajectory;
	
	RoboCompCommonBehavior::ParameterList worker_params;
	QMutex *worker_params_mutex;

};

#endif

