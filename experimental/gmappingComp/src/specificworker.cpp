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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx &mprx) : GenericWorker(mprx)
{
	mutex = new QMutex();
	active = false;
	interfacePoseWrite = &interfacePoseA;
	interfacePoseRead = &interfacePoseB;
	interfaceMapWrite = &interfaceMapA;
	interfaceMapRead = &interfaceMapB;
	std::setlocale(LC_ALL, "C");

	// GUI
	/*scene = new QGraphicsScene(QRect(0,0,1000,1000));
	view = new QGraphicsView(scene, this->frame);
	view->resize(this->frame->size());
	view->setRenderHints( QPainter::Antialiasing );
	
	view->show();*/
	widget->setFixedSize(WIDGETWIDTH, WIDGETWIDTH);
	QRect worldSize(-VISUALMAPWITDH / 2, VISUALMAPWITDH / 2, VISUALMAPWITDH, -VISUALMAPWITDH);
	map = new RCDrawRobot(worldSize, widget);
	map->show();

	v2DData.resize(1);

	// 	connect(map, SIGNAL(newLeftCoor(QPointF)), this, SLOT(newWorldCoor(QPointF)));
	connect(saveMapButton, SIGNAL(clicked()), this, SLOT(saveMap()));
	connect(loadMapButton, SIGNAL(clicked()), this, SLOT(loadMap()));
	connect(resetMapButton, SIGNAL(clicked()), this, SLOT(resetMap()));
	connect(txSB, SIGNAL(valueChanged(double)), this, SLOT(regenerateRT()));
	connect(tzSB, SIGNAL(valueChanged(double)), this, SLOT(regenerateRT()));
	connect(rySB, SIGNAL(valueChanged(double)), this, SLOT(regenerateRT()));

	connect(map, SIGNAL(iniMouseCoor(QPoint)), this, SLOT(iniMouseCoor(QPoint)));
	connect(map, SIGNAL(endMouseCoor(QPoint)), this, SLOT(endMouseCoor(QPoint)));
	
	worker_params_mutex = new QMutex();
	RoboCompCommonBehavior::Parameter aux;
	aux.editable = false;
	aux.type = "float";
	aux.value = "0";
	worker_params["frameRate"] = aux;

}

SpecificWorker::~SpecificWorker()
{}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	qDebug() << "setting params";
	this->params = params;
	initialize();
	timer.start(Period);

	active = true;
	return true;
}

void SpecificWorker::initialize()
{
	//PROCESSOR CREATION
	rInfo("Worker::Creating GridSlamProcessor");
	processor = new GridSlamProcessor();
	processed = false;
	//SENSOR MAP
	try
	{
		RoboCompGenericBase::TBaseState dd;
		laserData = laser_proxy->getLaserAndBStateData(dd);
	}
	catch (const Ice::Exception &ex)
	{
		qFatal("gmappingComp::initialize(): Error, no laser connection");
	}

	distances_laser = new double[laserData.size()];
	if (laserData.size() < 10)
		qFatal("Laser data size extremely small %ld", laserData.size());

	rs = new RangeSensor(std::string("FLASER"), laserData.size(),
	                     (laserData[laserData.size() - 1].angle - laserData[0].angle) / (laserData.size() - 1),
	                     OrientedPoint(0.23f, 0.f, 0.f), laserData[0].angle * 2.f,
	                     QString::fromStdString(params["GMapping.maxrange"].value).toDouble());

	sensorMap["FLASER"] = rs;

	rInfo("Worker::Setting sensor map");
	processor->setSensorMap(sensorMap);

	rInfo("Worker::Setting parameters in processor");
	//set the command line parameters
	processor->setMatchingParameters(QString::fromStdString(params["GMapping.maxUrange"].value).toDouble(),
	                                 QString::fromStdString(params["GMapping.maxrange"].value).toDouble(),
	                                 QString::fromStdString(params["GMapping.sigma"].value).toDouble(),
	                                 QString::fromStdString(params["GMapping.kernelSize"].value).toInt(),
	                                 QString::fromStdString(params["GMapping.lstep"].value).toDouble(),
	                                 QString::fromStdString(params["GMapping.astep"].value).toDouble(),
	                                 QString::fromStdString(params["GMapping.iterations"].value).toInt(),
	                                 QString::fromStdString(params["GMapping.lsigma"].value).toDouble(),
	                                 QString::fromStdString(params["GMapping.ogain"].value).toDouble(),
	                                 QString::fromStdString(params["GMapping.lskip"].value).toUInt());

	rInfo("Worker::Setting motion model parameters in processor");
	processor->setMotionModelParameters(QString::fromStdString(params["GMapping.srr"].value).toDouble(),
	                                    QString::fromStdString(params["GMapping.srt"].value).toDouble(),
	                                    QString::fromStdString(params["GMapping.str"].value).toDouble(),
	                                    QString::fromStdString(params["GMapping.stt"].value).toDouble());

	rInfo("Worker::Setting update distances parameters in processor");
	processor->setUpdateDistances(QString::fromStdString(params["GMapping.linearUpdate"].value).toDouble(),
	                              QString::fromStdString(params["GMapping.angularUpdate"].value).toDouble(),
	                              QString::fromStdString(params["GMapping.resampleThreshold"].value).toDouble());


	rInfo("Worker::Generate map in processor");
	processor->setgenerateMap((bool) QString::fromStdString(params["GMapping.generateMap"].value).toInt());
	printf("generate map: %d\n", (bool) QString::fromStdString(params["GMapping.generateMap"].value).toInt());

	xmin = QString::fromStdString(params["GMapping.xmin"].value).toDouble();
	xmax = QString::fromStdString(params["GMapping.xmax"].value).toDouble();
	ymin = QString::fromStdString(params["GMapping.ymin"].value).toDouble();
	ymax = QString::fromStdString(params["GMapping.ymax"].value).toDouble();


	auto s2d = [](std::string v) -> double { return QString::fromStdString(v).toDouble(); };
	txSB->setValue(s2d(params["GMapping.tx"].value));
	tzSB->setValue(s2d(params["GMapping.tz"].value));
	rySB->setValue(s2d(params["GMapping.ry"].value));
	regenerateRT();

// 	configGetString("", "GMapping.generateMap", aux.value,"true");
// 	params["GMapping.generateMap"] = aux;
// 	configGetString("", "GMapping.generateMap", aux.value,"true");
// 	params["GMapping.generateMap"] = aux;
// 	configGetString("", "GMapping.generateMap", aux.value,"true");
// 	params["GMapping.generateMap"] = aux;


	processor->setllsamplerange(QString::fromStdString(params["GMapping.llsamplerange"].value).toDouble());
	processor->setllsamplestep(QString::fromStdString(params["GMapping.llsamplestep"].value).toDouble());
	processor->setlasamplerange(QString::fromStdString(params["GMapping.lasamplerange"].value).toDouble());
	processor->setlasamplestep(QString::fromStdString(params["GMapping.lasamplestep"].value).toDouble());
	processor->setenlargeStep((bool) QString::fromStdString(params["GMapping.lasamplestep"].value).toInt());

	//bState update
	try
	{
		RoboCompGenericBase::TBaseState dd;
		laserData = laser_proxy->getLaserAndBStateData(dd);
	}
	catch (const Ice::Exception &ex)
	{
		qFatal("gmappingComp::initialize(): Error, no laser connection");
	}
	try
	{
		genericbase_proxy->getBaseState(bState);
	}
	catch (const Ice::Exception &ex)
	{
		qFatal("gmappingComp::initialize(): Error, no genericbase connection");
	}
		
	//INITIALIZATION
	rInfo("Worker::Initializing processor");
	int numParticles = QString::fromStdString(params["GMapping.particles"].value).toInt();

	// 	OrientedPoint OdomPose(bState.z/1000.f, bState.x/1000.f, bState.alpha);
	std::vector<OrientedPoint> initialPose;
	// 	QVec xg = QVec::uniformVector(numParticles, bState.z/1000.-2.,bState.z/1000.+2.);
	// 	QVec yg = QVec::uniformVector(numParticles, bState.x/1000.-2.,bState.x/1000.+2.);
	// 	QVec ag = QVec::uniformVector(numParticles, 0, 2.*M_PI);



	printf("params size: %s\n", params["GMapping.particles"].value.c_str());
	if (params["GMapping.Map"].value.size() > 0)
	{
		for (int i = 0; i < numParticles; i++)
		{
			// 	initialPose.push_back( OrientedPoint(xg[i],yg[i],ag[i]) );
			initialPose.push_back(OrientedPoint(0, 0, 0));
		}
		printf("loadMap\n");
		ScanMatcherMap *loadedMap = GridFastSlamMapHandling::loadMap(params["GMapping.Map"].value);
		printf("processor->init\n");
		processor->init(QString::fromStdString(params["GMapping.particles"].value).toInt(), xmin, ymin, xmax, ymax,
		                QString::fromStdString(params["GMapping.delta"].value).toDouble(), initialPose, *loadedMap);
		delete loadedMap;
	}
	else
	{
		for (int i = 0; i < numParticles; i++)
		{
			initialPose.push_back(OrientedPoint(0, 0, 0));
		}
		processor->init(QString::fromStdString(params["GMapping.particles"].value).toInt(), xmin, ymin, xmax, ymax,
		                QString::fromStdString(params["GMapping.delta"].value).toDouble(), initialPose);
	}

	nParticles = numParticles;

	QString rs = QString::fromStdString(params["GMapping.generateMap"].value).toLower();
	// 	qFatal("%s\n", rs.toStdString().c_str());
	if (rs == "true" or rs == "yes" or rs == "1" or rs == "y")
		registerScan = true;
	else
		registerScan = false;

	double delta = QString::fromStdString(params["GMapping.delta"].value).toDouble();
	int xSamples = (int) ((ymax - ymin) / delta);
	int zSamples = (int) ((xmax - xmin) / delta);
	gridMapBuffer = new uint8_t[xSamples * zSamples];
}

/// Last value of RangeSensor constructor should be changed. :-?
GMapping::RangeReading SpecificWorker::robocompWrapper(RoboCompGenericBase::TBaseState usedState)
{
	static QTime baseTime = QTime::currentTime();
	for (uint i = 0; i < laserData.size(); ++i)
	{
		//qDebug()<<laserData.size()<<i<<laserData[i].dist;
		if (laserData[i].dist < 100.)
			distances_laser[i] = 0.;
		else
			distances_laser[i] = laserData[i].dist / 1000.;
	}
	float t = (double) baseTime.elapsed() / 1000.;
	baseTime.restart();
	RangeReading rr(laserData.size(), distances_laser, rs, t);
	OrientedPoint p(usedState.z / 1000.f, usedState.x / 1000.f, usedState.alpha);
	rr.setPose(p);
	return rr;
}


void SpecificWorker::compute()
{
	static QTime lastDrawn = QTime::currentTime();
	static QTime reloj = QTime::currentTime();
	// 	static QTime lastSent = QTime::currentTime();
	// 	static QVec lastPosSent = QVec::vec3();
	// 	static float lastAngleSent = 0;

	// 	auto setLast = [](auto &bState, auto &lastSent, auto &lastPosSent, auto &lastAngleSent)
	// 	{
	// 		lastSent = QTime::currentTime();
	// 		lastPosSent = QVec::vec3(bState.correctedX, 0, bState.correctedZ);
	// 		lastAngleSent = bState.correctedAlpha;
	// 	};
	// 	auto shouldISend = [](auto &bState, auto &lastSent, auto &lastPosSent, auto &lastAngleSent)
	// 	{
	// 		const QVec p = QVec::vec3(bState.correctedX, 0, bState.correctedZ);
	// 		auto moved = fabs(bState.correctedAlpha-lastAngleSent)>0.01 or (lastPosSent-p).norm2()>8;
	// 		moved = moved and lastSent.elapsed() > 200;
	//
	// 		return moved or lastSent.elapsed() > 2000;
	// 	};

	Map<double, DoubleArray2D, false> *mymap = NULL;

	try
	{
		RoboCompGenericBase::TBaseState dd;
		laserData = laser_proxy->getLaserAndBStateData(dd);
		genericbase_proxy->getBaseState(bState);
		// qDebug()<<"base pose"<<bState.x<<bState.z<<bState.alpha;

		// This returns true when the algorithm effectively processes (the traveled path since the last processing is over a given threshold)
		// printf("register: %d\n", registerScan);
		processed = processor->processScan(robocompWrapper(bState), nParticles, registerScan);
		//for searching for the BEST PARTICLE INDEX
		best_idx = processor->getBestParticleIndex();
		// if you want to access to the PARTICLE VECTOR
		// const GridSlamProcessor::ParticleVector& particles = processor->getParticles();
		// remember to use a const reference, otherwise it copys the whole particles and maps
		if (processed or lastDrawn.elapsed() > 1000)
		{
			mymap = processor->getParticles()[best_idx].map.toDoubleMap();
		}
		if (processed)
		{
			OrientedPoint po = processor->getParticles()[best_idx].pose;
			RoboCompGenericBase::TBaseState estimatedPose;
			estimatedPose.x = po.y * 1000.;
			estimatedPose.z = po.x * 1000.;
			estimatedPose.alpha = po.theta;
			printf(" - - - - - - - - - PERFORMING CORRECTION - - - - - - - - - \n");
			printf("             estimated: (%f %f [%f])\n", estimatedPose.x, estimatedPose.z, estimatedPose.alpha);
			QVec init = QVec::vec3(estimatedPose.x, 0, estimatedPose.z);
			finalCorrection = (mapTransform * init.toHomogeneousCoordinates()).fromHomogeneousCoordinates();
			// 			finalCorrection.print("f1");
			// 			(mapTransform.invert() * QVec::vec4(correction.x, 0, correction.z, 1)).fromHomogeneousCoordinates().print("f1**-1");
			printf("%f %f   __   %f\n", finalCorrection(0), finalCorrection(2), estimatedPose.alpha - mapTransform_ry);


			cgrtopic_proxy->newCGRCorrection(0, bState.correctedX, bState.correctedZ, bState.correctedAlpha,
			                                 finalCorrection(0), finalCorrection(2),
			                                 estimatedPose.alpha - mapTransform_ry);


			printf("genericbase_proxy->correctOdometer(%f,  %f,  %f),  (%f,  %f,  %f)\n", bState.correctedX,
			       bState.correctedZ, bState.correctedAlpha, finalCorrection(0), finalCorrection(2),
			       estimatedPose.alpha - mapTransform_ry);
			// 			setLast(bState, lastSent, lastPosSent, lastAngleSent);
		}
		// 		else if (shouldISend(bState, lastSent, lastPosSent, lastAngleSent))
		// 		{
		// 			setLast(bState, lastSent, lastPosSent, lastAngleSent);
		// 			cgrtopic_proxy->newCGRPose(0, bState.correctedX, bState.correctedZ, bState.correctedAlpha);
		//			printf(bState.correctedX, bState.correctedZ, bState.correctedAlpha);
		// 		}

		v2DData[0] = ((double) processor->getneff()) / ((double) nParticles);
	}
	catch (const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	};


	if (mymap != NULL)
	{
		lastDrawn = QTime::currentTime();
		drawMap(mymap);
		drawAllLines();
		drawAllParticles(mymap);
		drawBestParticle(mymap);
		drawOdometry(mymap);
		map->update();
	}

	if (mymap != NULL)
		updateMap(mymap);
	delete mymap;

	worker_params_mutex->lock();
		//save framerate in params
		worker_params["frameRate"].value = std::to_string(reloj.restart()/1000.f);
	worker_params_mutex->unlock();
}

void SpecificWorker::updateMap(Map<double, DoubleArray2D, false> *mymap)
{
	// Get map bounding box
	double xMin, zMin, xMax, zMax;
	mymap->getSize(zMin, xMin, zMax, xMax);
	// Get best particle and set the pose
	const GridSlamProcessor::ParticleVector &particles = processor->getParticles();
	OrientedPoint po = particles[best_idx].pose;
	interfacePoseWrite->x = po.y;
	interfacePoseWrite->z = po.x;
	interfacePoseWrite->alpha = po.theta;
	double delta = QString::fromStdString(params["GMapping.delta"].value).toDouble();
	int32_t xSamples = (int32_t) ((xMax - xMin) / delta);
	int32_t zSamples = (int32_t) ((zMax - zMin) / delta);

	delete gridMapBuffer;
	gridMapBuffer = new uint8_t[xSamples * zSamples];
	memset(gridMapBuffer, 0, xSamples * zSamples);


	Point p;
	for (int iZ = 0; iZ < zSamples; iZ++)
	{
		p.x = ((double) (zSamples / 2 - iZ)) * delta;
		for (int iX = 0; iX < xSamples; iX++)
		{
			p.y = ((double) (iX - xSamples / 2)) * delta;
			if (p.x <= zMin or p.x >= zMax or p.y <= xMin or p.y >= xMax)
				continue;
			if (mymap->cell(p) > 0.01)
				gridMapBuffer[iZ * xSamples + iX] = 255;
			else
				gridMapBuffer[iZ * xSamples + iX] = 0;
		}
	}

	// Set interface data
	interfaceMapWrite->params.width = xSamples;
	interfaceMapWrite->params.height = zSamples;
	interfaceMapWrite->params.rect.minX = 1000. * xMin;
	interfaceMapWrite->params.rect.maxX = 1000. * xMax;
	interfaceMapWrite->params.rect.minZ = 1000. * zMin;
	interfaceMapWrite->params.rect.maxZ = 1000. * zMax;
	interfaceMapWrite->params.millimetersPerCell =
		(interfaceMapWrite->params.rect.maxX - interfaceMapWrite->params.rect.minX) / xSamples;
	if ((int64_t) interfaceMapWrite->data.size() != xSamples * zSamples)
		interfaceMapWrite->data.resize(xSamples * zSamples);
	memcpy(&(interfaceMapWrite->data[0]), gridMapBuffer, xSamples * zSamples);
	QMutexLocker locker(mutex);
	RoboCompSlamLaser::GridMap *tempM = interfaceMapWrite;
	interfaceMapWrite = interfaceMapRead;
	interfaceMapRead = tempM;
	RoboCompSlamLaser::Pose2D *tempP = interfacePoseWrite;
	interfacePoseWrite = interfacePoseRead;
	interfacePoseRead = tempP;
}

void SpecificWorker::drawMap(Map<double, DoubleArray2D, false> *mymap)
{
	map->drawLine(QLine(-10000, 0, 10000, 0), Qt::black, 25);
	map->drawLine(QLine(0, -10000, 0, 10000), Qt::black, 25);

	double xMin, yMin, xMax, yMax;
	mymap->getSize(xMin, yMin, xMax, yMax);

	double deltaX = (xMax - xMin) / WIDGETWIDTH, deltaY = (yMax - yMin) / WIDGETWIDTH;
	Point p;

	for (int i = 0; i < WIDGETWIDTH and p.y < yMax; ++i)
	{
		for (int j = 0; j < WIDGETWIDTH and p.x < xMax; ++j)
		{
			p.x = xMin + deltaX * j;
			p.y = yMin + deltaY * i;
			double v = mymap->cell(p);
			if (v >= 0)
			{
			//	qDebug()<<"pinta"<<p.x<<p.y;
				map->drawSquare(QPointF(p.y * 1000 - 5, p.x * 1000 - 5), 10, 10, Qt::darkBlue, true);
			//	drawSquare(QRect(p.y,p.x,100,100));
			}
		}
	}
}

void SpecificWorker::drawAllParticles(Map<double, DoubleArray2D, false> *mymap)
{
	const GridSlamProcessor::ParticleVector &particles = processor->getParticles();
	OrientedPoint pPose;
	for (uint k = 0; k < particles.size(); k++)
	{
		pPose = particles[k].pose;
		map->drawSquare(QPointF(pPose.y * 1000, pPose.x * 1000), 25, 25, Qt::cyan, true);
	}
}

void SpecificWorker::drawAllLines()
{
	for (auto lineToDraw : linesToDraw)
	{
		map->drawLine(
			QLine(lineToDraw.first.first, lineToDraw.first.second, lineToDraw.second.first, lineToDraw.second.second),
			Qt::red, 25);

	}
	// 	std::vector<std::pair<std::pair<float, float>>> linesToDraw;
	// 	const GridSlamProcessor::ParticleVector& particles = processor->getParticles();
	// 	OrientedPoint pPose;
	// 	for(uint k=0; k<particles.size(); k++)
	// 	{
	// 		pPose = particles[k].pose;
	// 		map->drawSquare(QPointF(pPose.y*1000, pPose.x*1000), 25, 25, Qt::cyan, true);
	// 	}
}


void SpecificWorker::drawBestParticle(Map<double, DoubleArray2D, false> *mymap)
{
	const GridSlamProcessor::ParticleVector &particles = processor->getParticles();

	OrientedPoint po = particles[best_idx].pose;


	QPoint center(1000. * po.y, 1000. * po.x);
	map->drawSquare(center, 100, 100, Qt::green, true);

	QLine l;
	l.setP1(center);
	QPoint p2a(-400. * sin(po.theta - 0.3), -400. * cos(po.theta - 0.3));
	l.setP2(center + p2a);
	map->drawLine(l, Qt::black);

	QPoint p2b(-400. * sin(po.theta + 0.3), -400. * cos(po.theta + 0.3));
	l.setP2(center + p2b);
	map->drawLine(l, Qt::blue);

	map->drawSquare(QPointF(po.y * 1000, po.x * 1000), 100, 100, Qt::red, true);
}

void SpecificWorker::drawOdometry(Map<double, DoubleArray2D, false> *mymap)
{
	QPoint center(bState.x, bState.z);
	map->drawSquare(center, 100, 100, Qt::green, true);

	QLine l;
	l.setP1(center);
	QPoint p2a(-400. * sin(bState.alpha - 0.3), -400. * cos(bState.alpha - 0.3));
	l.setP2(center + p2a);
	map->drawLine(l, Qt::black);

	QPoint p2b(-400. * sin(bState.alpha + 0.3), -400. * cos(bState.alpha + 0.3));
	l.setP2(center + p2b);
	map->drawLine(l, Qt::blue);
}

/**
 *	Conditionally enqueues new robot's pose into the list of poses that describes the robot's trajectory.
 *	If the last robot's position is not
 */
void SpecificWorker::queueLastPose()
{
	OrientedPoint po = processor->getParticles()[best_idx].pose;
	if (trajectory.size() > 0)
	{
		float d;
		if ((d = sqrt(
			pow(po.x - trajectory.last()->operator[](0), 2) + pow(po.y - trajectory.last()->operator[](1), 2))) < 0.1)
			return;
	}

	QVector<float> *tp;
	if (trajectory.size() < 300)
	{
		tp = new QVector<float>;
		tp->resize(2);
	}
	else
	{
		tp = trajectory.takeAt(0);
	}
	trajectory.push_back(tp);
	tp->operator[](0) = po.x;
	tp->operator[](1) = po.y;
}

void SpecificWorker::saveMap()
{
	QString fn = QFileDialog::getSaveFileName(this, "*", ".", "Maps (*.map)");
	if (fn != "")
		saveMap(fn.toStdString());
}

void SpecificWorker::loadMap()
{
	QString fn = QFileDialog::getOpenFileName(this, "", ".", "Maps (*.map)");

	if (fn != "")
	{
		ScanMatcherMap *loadedMap = GridFastSlamMapHandling::loadMap(fn.toStdString());
		// 	OrientedPoint OdomPose(bState.z/1000.f, bState.x/1000.f, bState.alpha);
		std::vector<OrientedPoint> initialPose;
		QVec xg = QVec::uniformVector(nParticles, bState.z / 1000. - 1., bState.z / 1000. + 1.);
		QVec yg = QVec::uniformVector(nParticles, bState.x / 1000. - 1., bState.x / 1000. + 1.);
		QVec ag = QVec::uniformVector(nParticles, -M_PI, M_PI);

		for (int i = 0; i < nParticles; i++)
		{
			initialPose.push_back(OrientedPoint(xg[i], yg[i], ag[i]));
		}
		processor->init(nParticles, xmin, ymin, xmax, ymax,
		                QString::fromStdString(params["GMapping.delta"].value).toDouble(), initialPose, *loadedMap);
	}
}


bool SpecificWorker::saveMap(const std::string &path)
{
	try
	{
		GridFastSlamMapHandling::saveMap(processor->getParticles()[best_idx].map, path);
		return true;
	}
	catch (...)
	{
		return false;
	}
}


/*
void SpecificWorker::newWorldCoor(QPointF p)
{
	if (action_cb->currentText() == "Set position")
	{
	
		printf(" - - - - - - - - -   PERFORMING RESET   - - - - - - - - - \n");
		int numParticles = QString::fromStdString(params["GMapping.particles"].value).toInt();
	// 	OrientedPoint OdomPose(p.y()/1000.f, p.x()/1000.f, 0.);
		std::vector<OrientedPoint> initialPose;
		QVec xg = QVec::uniformVector(numParticles, (-p.y()/1000.)-0.5, (-p.y()/1000.)+0.5);
		QVec yg = QVec::uniformVector(numParticles, (+p.x()/1000.)-0.5, (+p.x()/1000.)+0.5);
		QVec ag = QVec::uniformVector(numParticles, -M_PI, M_PI);

		for(int i=0; i< numParticles; i++)
		{
			initialPose.push_back( OrientedPoint(xg[i], yg[i], ag[i]) );
		}

		if (params["GMapping.Map"].value.size() > 0)
		{
			ScanMatcherMap* loadedMap = GridFastSlamMapHandling::loadMap(params["GMapping.Map"].value);
			printf("processor->init(%d, %g, %g, %g, %g, %g, POSES)\n", QString::fromStdString(params["GMapping.particles"].value).toInt(), xmin, ymin, xmax, ymax, QString::fromStdString(params["GMapping.delta"].value).toDouble());

			processor->init(QString::fromStdString(params["GMapping.particles"].value).toInt(), xmin, ymin, xmax, ymax, QString::fromStdString(params["GMapping.delta"].value).toDouble(), initialPose, *loadedMap);
			delete loadedMap;
		}
		action_cb->setCurrentIndex(0);
	}
	else if (action_cb->currentText() == "Set lines")
	{
		printf(" - - - - - - - - -   Setting LINES   - - - - - - - - - \n");
		
	}
}
*/
void SpecificWorker::resetMap()
{
// 	OrientedPoint OdomPose(bState.z/1000.f, bState.x/1000.f, bState.alpha);
	std::vector<OrientedPoint> initialPose;
	QVec xg = QVec::uniformVector(nParticles, bState.z / 1000. - 1., bState.z / 1000. + 1.);
	QVec yg = QVec::uniformVector(nParticles, bState.x / 1000. - 1., bState.x / 1000. + 1.);
	QVec ag = QVec::uniformVector(nParticles, 0, 2. * M_PI);

	for (int i = 0; i < nParticles; i++)
	{
		initialPose.push_back(OrientedPoint(xg[i], yg[i], ag[i]));
	}

	processor->init(nParticles, xmin, ymin, xmax, ymax,
	                QString::fromStdString(params["GMapping.delta"].value).toDouble(), initialPose);
}

void SpecificWorker::getWholeGrid(GridMap &map, Pose2D &pose)
{
	QMutexLocker locker(mutex);
	map = *interfaceMapRead;
	pose = *interfacePoseRead;

}

void SpecificWorker::initializeRobotPose(const Pose2D &pose)
{
	// 	newWorldCoor(QPointF(pose.x, pose.z));
}

void SpecificWorker::getPartialGrid(const MapRect &rect, GridMap &map, Pose2D &pose)
{
	QMutexLocker locker(mutex);
}

/*
void SpecificWorker::drawSquare(const QRect &rect)
{
	scene->addRect(rect, QPen(Qt::darkBlue), QBrush(Qt::darkBlue));
//	QPainter painter ( map );
	//painter.setWindow ( worldSize );
	painter.setRenderHint(QPainter::HighQualityAntialiasing);
	painter.setBrush ( Qt::darkBlue );
	QPen pen = painter.pen();
	pen.setColor( Qt::darkBlue  );
	pen.setWidth( 100 );
	painter.setPen(pen);
	painter.drawRect( rect );
	
}
*/

void SpecificWorker::regenerateRT()
{
	mapTransform = RTMat(0, rySB->value(), 0, QVec::vec3(txSB->value(), 0, tzSB->value())).invert();
	mapTransform_ry = rySB->value();
	// 	finalCorrection = ((*mapTransform) * QVec::vec4(correction.x, 0, correction.z, 1)).fromHomogeneousCoordinates();
	// 	printf("%f %f   __   %f\n", finalCorrection(0), finalCorrection(2), correction.alpha+mapTransform_ry);
	// 	printf("regenerateRT %f %f %f\n", rySB->value(), txSB->value(), tzSB->value());
}


RoboCompCommonBehavior::ParameterList SpecificWorker::getWorkerParams()
{
	QMutexLocker locker(worker_params_mutex);
	return worker_params;
}

