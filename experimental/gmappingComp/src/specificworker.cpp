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
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	mutex = new QMutex();
	active = false;
	interfacePoseWrite = &interfacePoseA;
	interfacePoseRead = &interfacePoseB;
	interfaceMapWrite = &interfaceMapA;
	interfaceMapRead = &interfaceMapB;

// GUI
	/*scene = new QGraphicsScene(QRect(0,0,1000,1000));
	view = new QGraphicsView(scene, this->frame);
	view->resize(this->frame->size());
	view->setRenderHints( QPainter::Antialiasing );
	
	view->show();*/
	QRect worldSize(-10000,10000,20000,-20000);
	map = new RCDraw(worldSize, this->frame);
	map->show();
//	connect(map, SIGNAL(newWorldCoor(QPointF)), this, SLOT(newWorldCoor(QPointF)));
	v2DData.resize(1);
	connect(saveMapButton,SIGNAL(clicked()), this, SLOT(saveMap()));
	connect(loadMapButton,SIGNAL(clicked()), this, SLOT(loadMap()));
	connect(resetMapButton,SIGNAL(clicked()), this, SLOT(resetMap()));
	
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	qDebug()<<"setting params";
	this->params = params;
	initialize();
	timer.start(Period);

	active = true;
	return true;
}


/**
* \brief Initialization method
*/
void SpecificWorker::initialize()
{
	//PROCESSOR CREATION
	rInfo("Worker::Creating GridSlamProcessor");
	processor = new GridSlamProcessor();
	processed = false;
	//SENSOR MAP
	try
	{
		RoboCompDifferentialRobot::TBaseState dd;
		laserData = laser_proxy->getLaserAndBStateData(dd);
	}
	catch(const Ice::Exception &ex)
	{
			qFatal("gmappingComp::initialize(): Error, no laser connection");
	}

	distances_laser = new double[laserData.size()];
	if (laserData.size()<10)
		qFatal("Laser data size extremely small %ld", laserData.size());

	rs = new RangeSensor(std::string("FLASER"), laserData.size(), (laserData[laserData.size()-1].angle-laserData[0].angle)/(laserData.size()-1), OrientedPoint(0.f,0.f,0.f), laserData[0].angle*2.f, QString::fromStdString(params["GMapping.maxrange"].value).toDouble());

	sensorMap["FLASER"] = rs;

	rInfo("Worker::Setting sensor map");
	processor->setSensorMap(sensorMap);

	rInfo("Worker::Setting parameters in processor");
	//set the command line parameters
	processor->setMatchingParameters( QString::fromStdString(params["GMapping.maxUrange"].value).toDouble(),
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
	processor->setgenerateMap((bool)QString::fromStdString(params["GMapping.generateMap"].value).toInt());
	printf("generate map: %d\n", (bool)QString::fromStdString(params["GMapping.generateMap"].value).toInt());

	xmin = QString::fromStdString(params["GMapping.xmin"].value).toDouble();
	xmax = QString::fromStdString(params["GMapping.xmax"].value).toDouble();
	ymin = QString::fromStdString(params["GMapping.ymin"].value).toDouble();
	ymax = QString::fromStdString(params["GMapping.ymax"].value).toDouble();


	processor->setllsamplerange(QString::fromStdString(params["GMapping.llsamplerange"].value).toDouble());
	processor->setllsamplestep(QString::fromStdString(params["GMapping.llsamplestep"].value).toDouble());
	processor->setlasamplerange(QString::fromStdString(params["GMapping.lasamplerange"].value).toDouble());
	processor->setlasamplestep(QString::fromStdString(params["GMapping.lasamplestep"].value).toDouble());
	processor->setenlargeStep((bool)QString::fromStdString(params["GMapping.lasamplestep"].value).toInt());

	//bState update
	try
	{
		RoboCompDifferentialRobot::TBaseState dd;
		laserData = laser_proxy->getLaserAndBStateData(dd);
		omnirobot_proxy->getBaseState(bState);
	}
	catch(const Ice::Exception &ex)
	{
		qFatal("gmappingComp::initialize(): Error, no laser connection");
	}
	
	//INITIALIZATION
	rInfo("Worker::Initializing processor");
	int numParticles = QString::fromStdString(params["GMapping.particles"].value).toInt();

	OrientedPoint OdomPose(bState.z/1000.f, bState.x/1000.f, bState.alpha);
	std::vector<OrientedPoint> initialPose;
	QVec xg = QVec::uniformVector(numParticles, bState.z/1000.-2.,bState.z/1000.+2.);
	QVec yg = QVec::uniformVector(numParticles, bState.x/1000.-2.,bState.x/1000.+2.);
	QVec ag = QVec::uniformVector(numParticles, 0, 2.*M_PI);


	for(int i=0; i< numParticles; i++)
	{
		initialPose.push_back( OrientedPoint(xg[i],yg[i],ag[i]) );
	}

	printf("params size: %s\n", params["GMapping.particles"].value.c_str());
	if (params["GMapping.Map"].value.size() > 0)
	{
		ScanMatcherMap* loadedMap = GridFastSlamMapHandling::loadMap(params["GMapping.Map"].value);
		processor->init(QString::fromStdString(params["GMapping.particles"].value).toInt(), xmin, ymin, xmax, ymax, QString::fromStdString(params["GMapping.delta"].value).toDouble(), initialPose, *loadedMap);
		delete loadedMap;
	}
	else
	{
		processor->init(QString::fromStdString(params["GMapping.particles"].value).toInt(), xmin, ymin, xmax, ymax, QString::fromStdString(params["GMapping.delta"].value).toDouble(), initialPose);
	}

	nParticles=numParticles;

	QString rs = QString::fromStdString(params["GMapping.generateMap"].value).toLower();
	if (rs == "true" or rs == "yes" or rs == "1" or rs == "y")
		registerScan = true;
	else
		registerScan = false;

	double delta = QString::fromStdString(params["GMapping.delta"].value).toDouble();
	int xSamples = (int)((ymax-ymin)/delta);
	int zSamples = (int)((xmax-xmin)/delta);
	gridMapBuffer = new uint8_t[xSamples*zSamples];
}

/// Last value of RangeSensor constructor should be changed. :-?
GMapping::RangeReading SpecificWorker::robocompWrapper(RoboCompOmniRobot::TBaseState usedState)
{
	static QTime baseTime = QTime::currentTime();
	for (uint i=0; i<laserData.size();++i)
	{
//qDebug()<<laserData.size()<<i<<laserData[i].dist;
		if (laserData[i].dist<300.)
			distances_laser[i] = 40.;
		else
			distances_laser[i] = laserData[i].dist/1000.;
	}
	float t = (double)baseTime.elapsed()/1000.;
	baseTime.restart();
	RangeReading rr(laserData.size(), distances_laser, rs, t);
	OrientedPoint p(usedState.z/1000.f, usedState.x/1000.f, usedState.alpha);
	rr.setPose( p );
	return rr;
}




void SpecificWorker::compute()
{
	Map<double, DoubleArray2D, false>* mymap=NULL;

	try
	{
		RoboCompDifferentialRobot::TBaseState dd;
		laserData = laser_proxy->getLaserAndBStateData(dd);
		omnirobot_proxy->getBaseState(bState);
qDebug()<<"laser data: "<<laserData.size();
qDebug()<<"base pose"<<bState.x<<bState.z<<bState.alpha;		

		// This returns true when the algorithm effectively processes (the traveled path since the last processing is over a given threshold)
		processed = processor->processScan(robocompWrapper(bState), nParticles, registerScan);
//  		if (processed)
		{
			//for searching for the BEST PARTICLE INDEX
			best_idx=processor->getBestParticleIndex();
			// if you want to access to the PARTICLE VECTOR
			// const GridSlamProcessor::ParticleVector& particles = processor->getParticles();
			// remember to use a const reference, otherwise it copys the whole particles and maps

			mymap = processor->getParticles()[best_idx].map.toDoubleMap();
			OrientedPoint po = processor->getParticles()[best_idx].pose;
			RoboCompOmniRobot::TBaseState estimatedPose;
			estimatedPose.x=po.y*1000.;
			estimatedPose.z=po.x*1000.;
			estimatedPose.alpha=po.theta;
			printf(" - - - - - - - - - PERFORMING CORRECTION - - - - - - - - - \n");
			RoboCompOmniRobot::TBaseState correction;
			correction.x     = estimatedPose.x     ;//- bState.correctedX;
			correction.z     = estimatedPose.z     ;//- bState.correctedZ;
			correction.alpha = estimatedPose.alpha ;//- bState.correctedAlpha;
			printf("             estimated: (%f %f [%f])\n", estimatedPose.x, estimatedPose.z, estimatedPose.alpha);
			printf("             corrected: (%f %f [%f])\n", bState.correctedX, bState.correctedZ, bState.correctedAlpha);
			printf("            correction: (%f %f [%f])\n", correction.x, correction.z, correction.alpha);
			omnirobot_proxy->correctOdometer(correction.x, correction.z, correction.alpha);
			printf("omnirobot_proxy->correctOdometer(%f,  %f,  %f)\n", correction.x, correction.z, correction.alpha);

			v2DData[0]=((double) processor->getneff())/((double) nParticles);
		}
	}
	catch (const Ice::Exception & ex)
	{
		std::cout << ex << std::endl;
	};


	if (mymap != NULL)
	{
qDebug()<<"drawign";
		drawMap(mymap);
		drawAllParticles(mymap);
		drawBestParticle(mymap);
		drawOdometry(mymap);
		map->update();
	}

	if (mymap != NULL)
		updateMap(mymap);
	delete mymap;

}

void SpecificWorker::updateMap(Map<double, DoubleArray2D, false>* mymap)
{
	// Get map bounding box
	double xMin, zMin, xMax, zMax;
	mymap->getSize(zMin, xMin, zMax, xMax);
	// Get best particle and set the pose
	const GridSlamProcessor::ParticleVector& particles = processor->getParticles();
	OrientedPoint po = particles[best_idx].pose;
	interfacePoseWrite->x = po.y;
	interfacePoseWrite->z = po.x;
	interfacePoseWrite->alpha = po.theta;
	double delta = QString::fromStdString(params["GMapping.delta"].value).toDouble();
	int32_t xSamples = (int32_t)((xMax-xMin)/delta);
	int32_t zSamples = (int32_t)((zMax-zMin)/delta);

	delete gridMapBuffer;
	gridMapBuffer = new uint8_t[xSamples*zSamples];
	memset(gridMapBuffer, 0, xSamples*zSamples);


	Point p;
	for (int iZ = 0; iZ<zSamples; iZ++)
	{
		p.x = ((double)(zSamples/2 - iZ)) * delta;
		for (int iX = 0; iX<xSamples; iX++)
		{
			p.y = ((double)(iX - xSamples/2)) * delta;
			if (p.x<=zMin or p.x>=zMax or p.y<=xMin or p.y>=xMax)
				continue;
			if (mymap->cell(p)>0.01)
				gridMapBuffer[iZ*xSamples+iX] = 255;
			else
				gridMapBuffer[iZ*xSamples+iX] = 0;
		}
	}

	// Set interface data
	interfaceMapWrite->params.width = xSamples;
	interfaceMapWrite->params.height = zSamples;
	interfaceMapWrite->params.rect.minX = 1000.*xMin;
	interfaceMapWrite->params.rect.maxX = 1000.*xMax;
	interfaceMapWrite->params.rect.minZ = 1000.*zMin;
	interfaceMapWrite->params.rect.maxZ = 1000.*zMax;
	interfaceMapWrite->params.millimetersPerCell = (interfaceMapWrite->params.rect.maxX-interfaceMapWrite->params.rect.minX)/xSamples;
	if ((int64_t)interfaceMapWrite->data.size()!=xSamples*zSamples)
		interfaceMapWrite->data.resize(xSamples*zSamples);
	memcpy(&(interfaceMapWrite->data[0]), gridMapBuffer, xSamples*zSamples);
	QMutexLocker locker(mutex);
	RoboCompSlamLaser::GridMap  *tempM = interfaceMapWrite;  interfaceMapWrite  = interfaceMapRead;  interfaceMapRead  = tempM;
	RoboCompSlamLaser::Pose2D   *tempP = interfacePoseWrite; interfacePoseWrite = interfacePoseRead; interfacePoseRead = tempP;
}

void SpecificWorker::drawMap(Map<double, DoubleArray2D, false>* mymap)
{
	double xMin, yMin, xMax, yMax;
	mymap->getSize(xMin, yMin, xMax, yMax);

	double deltaX=(xMax-xMin)/WIDGETWIDTH, deltaY=(yMax-yMin)/WIDGETWIDTH;
	Point p;

	for (int i=0; i<WIDGETWIDTH and p.y<yMax; ++i)
	{
		for (int j=0; j<WIDGETWIDTH and p.x<xMax; ++j)
		{
			p.x = xMin + deltaX*j;
			p.y=  yMin + deltaY*i;
			double v = mymap->cell(p);
			if (v>=0)
			{
// 				qDebug()<<"pinta"<<p.x<<p.y;
				map->drawSquare(QPointF(p.y*1000,p.x*1000), 100, 100, Qt::darkBlue, true);
//				drawSquare(QRect(p.y,p.x,100,100));
			}
		}
	}
}

void SpecificWorker::drawAllParticles(Map<double, DoubleArray2D, false>* mymap)
{
	const GridSlamProcessor::ParticleVector& particles = processor->getParticles();
	OrientedPoint pPose;
	for(uint k=0; k<particles.size(); k++)
	{
		pPose = particles[k].pose;
		map->drawSquare(QPointF(pPose.y*1000, pPose.x*1000), 500, 500, Qt::cyan, true);
	}
}

void SpecificWorker::drawBestParticle(Map<double, DoubleArray2D, false>* mymap)
{
	const GridSlamProcessor::ParticleVector& particles = processor->getParticles();

	OrientedPoint po = particles[best_idx].pose;
	map->drawSquare(QPointF(po.y*1000, po.x*1000), 600, 600, Qt::red, true);
}

void SpecificWorker::drawOdometry(Map<double, DoubleArray2D, false>* mymap)
{
	map->drawSquare(QPointF(bState.x, bState.z), 600, 600, Qt::green, true);
	map->drawSquare(QPointF(bState.correctedX, bState.correctedZ), 400, 400, Qt::black, true);
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
		if ((d=sqrt(pow(po.x-trajectory.last()->operator[](0),2) + pow(po.y-trajectory.last()->operator[](1),2))) < 0.1)
			return;
	}

	QVector<float> *tp;
	if (trajectory.size()<300)
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
	if(fn!="")
		saveMap(fn.toStdString());
}

void SpecificWorker::loadMap()
{
	QString fn = QFileDialog::getOpenFileName(this,"",".", "Maps (*.map)");

	if(fn!="")
	{
		ScanMatcherMap* loadedMap = GridFastSlamMapHandling::loadMap(fn.toStdString());
		OrientedPoint OdomPose(bState.z/1000.f, bState.x/1000.f, bState.alpha);
		std::vector<OrientedPoint> initialPose;
		QVec xg = QVec::uniformVector(nParticles, bState.z/1000.-1.,bState.z/1000.+1.);
		QVec yg = QVec::uniformVector(nParticles, bState.x/1000.-1.,bState.x/1000.+1.);
		QVec ag = QVec::uniformVector(nParticles, -M_PI, M_PI);

		for(int i=0; i<nParticles; i++)
		{
			initialPose.push_back( OrientedPoint(xg[i],yg[i],ag[i]) );
		}
		processor->init(nParticles, xmin, ymin, xmax, ymax, QString::fromStdString(params["GMapping.delta"].value).toDouble(), initialPose, *loadedMap);
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



void SpecificWorker::newWorldCoor(QPointF p)
{
	RoboCompOmniRobot::TBaseState sentState;
	sentState.x = sentState.correctedX = p.x();
	sentState.z = sentState.correctedZ = p.y();
	sentState.alpha = sentState.correctedAlpha = 0;
	omnirobot_proxy->setOdometer(sentState);

	printf(" - - - - - - - - -   PERFORMING RESET   - - - - - - - - - \n");
	int numParticles = QString::fromStdString(params["GMapping.particles"].value).toInt();
	OrientedPoint OdomPose(p.y()/1000.f, p.x()/1000.f, 0.);
	std::vector<OrientedPoint> initialPose;
	QVec xg = QVec::uniformVector(numParticles, p.y()/1000.-0.75, p.y()/1000.+0.75);
	QVec yg = QVec::uniformVector(numParticles, p.x()/1000.-0.75, p.x()/1000.+0.75);
	QVec ag = QVec::uniformVector(numParticles, -M_PI, M_PI);

	for(int i=0; i< numParticles; i++)
	{
		initialPose.push_back( OrientedPoint(xg[i],yg[i],ag[i]) );
	}

	if (params["GMapping.Map"].value.size() > 0)
	{
		ScanMatcherMap* loadedMap = GridFastSlamMapHandling::loadMap(params["GMapping.Map"].value);
		printf("processor->init(%d, %g, %g, %g, %g, %g, POSES)\n", QString::fromStdString(params["GMapping.particles"].value).toInt(), xmin, ymin, xmax, ymax, QString::fromStdString(params["GMapping.delta"].value).toDouble());

		processor->init(QString::fromStdString(params["GMapping.particles"].value).toInt(), xmin, ymin, xmax, ymax, QString::fromStdString(params["GMapping.delta"].value).toDouble(), initialPose, *loadedMap);
		delete loadedMap;
	}
}

void SpecificWorker::resetMap()
{
	OrientedPoint OdomPose(bState.z/1000.f, bState.x/1000.f, bState.alpha);
	std::vector<OrientedPoint> initialPose;
	QVec xg = QVec::uniformVector(nParticles, bState.z/1000.-1.,bState.z/1000.+1.);
	QVec yg = QVec::uniformVector(nParticles, bState.x/1000.-1.,bState.x/1000.+1.);
	QVec ag = QVec::uniformVector(nParticles, 0, 2.*M_PI);

	for(int i=0; i< nParticles; i++)
	{
		initialPose.push_back( OrientedPoint(xg[i],yg[i],ag[i]) );
	}

	processor->init(nParticles, xmin, ymin, xmax, ymax, QString::fromStdString(params["GMapping.delta"].value).toDouble(), initialPose);
}

void SpecificWorker::getWholeGrid(GridMap &map, Pose2D &pose)
{
	QMutexLocker locker(mutex);
	map  = *interfaceMapRead;
	pose = *interfacePoseRead;

}

void SpecificWorker::initializeRobotPose(const Pose2D &pose)
{
	newWorldCoor(QPointF(pose.x, pose.z));
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



