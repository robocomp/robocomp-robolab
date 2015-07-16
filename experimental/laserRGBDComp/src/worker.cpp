/*
 *    Copyright (C) 2006-2011 by RoboLab - University of Extremadura
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
#include "worker.h"
/**
* \brief Default constructor
*/

// #define USE_EXTENSION
#define EXTENSION_RANGE 6.283185307179586
// #define EXTENSION_RANGE 3.1
Worker::Worker(RoboCompOmniRobot::OmniRobotPrx omnirobotprx, RoboCompJointMotor::JointMotorPrx jointmotorprx, RoboCompLaser::LaserPrx laserprx, WorkerConfig &cfg) : QObject()
{
	omnirobot = omnirobotprx;
	jointmotor = jointmotorprx;
	laser = laserprx;

	rgbds        = cfg.rgbdsVec;
	base         = QString::fromStdString(cfg.laserBaseID);
	actualLaserID= QString::fromStdString(cfg.actualLaserID);
	minHeight    = cfg.minHeight;
	minHeightNeg = cfg.minHeightNeg;
	maxHeight    = cfg.maxHeight;
	LASER_SIZE   = cfg.LASER_SIZE;
	MIN_LENGTH   = cfg.MIN_LENGTH;
	maxLength    = cfg.maxLength;
#ifdef USE_EXTENSION
	FOV          = EXTENSION_RANGE;
#else
	FOV          = cfg.FOV;
#endif
	localFOV     = cfg.FOV;
	updateJoint  = cfg.updateJoint;
	DECIMATION_LEVEL = cfg.DECIMATION_LEVEL;

	mutex = new QMutex();

	innerModel = new InnerModel(cfg.xmlpath);

	connect(&timer, SIGNAL(timeout()), this, SLOT(compute()));

	/// Resize and initialization
	laserDataR = new RoboCompLaser::TLaserData;
	laserDataW = new RoboCompLaser::TLaserData;
	laserDataR->resize(LASER_SIZE);
	laserDataW->resize(LASER_SIZE);
	for (int32_t i=0; i<LASER_SIZE; i++)
	{
		(*laserDataR)[i].angle = (float(i)-(LASER_SIZE/2.))*(cfg.FOV/LASER_SIZE);
		(*laserDataW)[i].angle = (float(i)-(LASER_SIZE/2.))*(cfg.FOV/LASER_SIZE);
	}
	printf("Direct field of view < %f -- %f >\n", (*laserDataR)[0].angle, (*laserDataR)[laserDataR->size()-1].angle);

	RoboCompOmniRobot::TBaseState oState;
	try { omnirobot->getBaseState(oState); }
	catch (Ice::Exception e) { qDebug()<<"error talking to base"<<e.what(); }
	innerModel->updateTransformValues("robot", bStateOut.x, 0, bStateOut.z, 0, bStateOut.alpha,0);
	bState.x     = oState.x;
	bState.z     = oState.z;
	bState.alpha = oState.alpha;
	extended = new ExtendedRangeSensor(*laserDataR, bState, innerModel, EXTENSION_RANGE, maxLength);
	extended->update(*laserDataR);

	confData.staticConf = 1;
#ifdef USE_EXTENSION
	confData.maxMeasures = extended->size();
	confData.iniRange = extended->getData(0).angle;
	confData.endRange = extended->getData(extended->size()-1).angle;
#else
	confData.maxMeasures = LASER_SIZE;
	confData.iniRange = (*laserDataR)[0].angle;
	confData.endRange = (*laserDataR)[LASER_SIZE-1].angle;
#endif
	confData.maxDegrees = FOV;
	confData.maxRange = maxLength;
	confData.minRange = MIN_LENGTH;
	confData.angleRes = cfg.FOV/LASER_SIZE;//confData.maxDegrees/confData.maxMeasures;
	confData.driver = "simulated from depth map";
	confData.device = "rgbd";

	compute();
	timer.start(200);
}

/**
* \brief Default destructor
*/
Worker::~Worker()
{

}
///Common Behavior

void Worker::killYourSelf()
{
	rDebug("Killing myself");
	emit kill();
	exit(1);
}
/**
* \brief Change compute period
* @param per Period in ms
*/
void Worker::setPeriod(int p)
{
	rDebug("Period changed"+QString::number(p));
	Period = p*1000;
}
/**
* \brief
* @param params_ Parameter list received from monitor thread
*/
bool Worker::setParams(RoboCompCommonBehavior::ParameterList params_)
{
	return true;
}


int32_t Worker::angle2bin(float ang)
{
	while (ang>M_PI)  ang -= 2.*M_PI;
	while (ang<-M_PI) ang += 2.*M_PI;

	float ret;
	ret = ang * (LASER_SIZE/localFOV);
	ret = ret + LASER_SIZE/2.;
	return int32_t(ret);
}

/**
* \brief Thread method
*/
// #define STORE_POINTCLOUDS_AND_EXIT

void Worker::compute()
{
	/// Clear laser measurement
	for (int32_t i=0; i<LASER_SIZE; ++i)
	{
		(*laserDataW)[i].dist = maxLength;
	}


	/// Update InnerModel with joint information
	if (updateJoint)
	{
		RoboCompJointMotor::MotorStateMap motorMap;
		try
		{
			jointmotor->getAllMotorState(motorMap);
			for (RoboCompJointMotor::MotorStateMap::iterator it=motorMap.begin(); it!=motorMap.end(); ++it)
			{
				innerModel->updateJointValue(it->first.c_str(), it->second.pos);
			}
		}
		catch (const Ice::Exception &ex)
		{
			cout << "Can't connect to jointMotor: " << ex << endl;
		}
	}
	else
	{
		printf("not using joint\n");
	}

	/// FOR EACH OF THE CONFIGURED PROXIES
// 	#pragma omp parallel for
	for (uint r=0; r<rgbds.size(); ++r)
	{
#ifdef STORE_POINTCLOUDS_AND_EXIT
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // PCL
#endif

		if (rgbds[r].bus == true) /// If the proxy is a bus
		{
			/// FOR EACH OF THE CAMERAS OF THE BUS
			RoboCompRGBDBus::CameraList clist;
			clist.resize(1);
			RoboCompRGBDBus::CameraParamsMap::iterator iter;
			for (iter=rgbds[r].cameras.begin(); iter!=rgbds[r].cameras.end(); iter++)
			{
				if (iter->first == rgbds[r].id)
				{
					clist[0] = iter->first;
					RoboCompRGBDBus::ImageMap images;

					try
					{
						if (DECIMATION_LEVEL == 0)
							rgbds[r].proxyRGBDBus->getImages(clist,images);
						else
							rgbds[r].proxyRGBDBus->getDecimatedImages(clist, DECIMATION_LEVEL, images);
					}
					catch (const Ice::Exception &ex)
					{
						cout << "Can't connect to rgbd: " << ex << endl;
						continue;
					}

					/// Get the corresponding (stored) protocloud
					RoboCompRGBDBus::PointCloud pointCloud = rgbds[r].protoPointClouds[clist[0]];
					/// Multiply the protocloud by the depth
					for (uint32_t pi=0; pi<pointCloud.size(); pi++)
					{
						pointCloud[pi].x *= images[iter->first].depthImage[pi];
						pointCloud[pi].y *= images[iter->first].depthImage[pi];
						pointCloud[pi].z  = images[iter->first].depthImage[pi];
					}

					/// Inserts the resulting points in the virtual laser
					RTMat TR = innerModel->getTransformationMatrix(base, QString::fromStdString(iter->first));
#ifdef STORE_POINTCLOUDS_AND_EXIT
					cloud->points.resize(pointCloud.size());
#endif
					for (uint32_t ioi=0; ioi<pointCloud.size(); ioi+=3)
					{
						if ((not isnan(images[iter->first].depthImage[ioi])) and images[iter->first].depthImage[ioi] > 10)
						{
							QVec p = (TR * QVec::vec4(pointCloud[ioi].x, pointCloud[ioi].y, pointCloud[ioi].z, 1)).fromHomogeneousCoordinates();
#ifdef STORE_POINTCLOUDS_AND_EXIT
							cloud->points[ioi].x =  p(0)/1000;
							cloud->points[ioi].y =  p(1)/1000;
							cloud->points[ioi].z = -p(2)/1000;
#endif
							if ( (p(1)>=minHeight and p(1)<=maxHeight) /* or (p(1)<minHeightNeg) */)
							{
								p(1) = 0;
								float d = sqrt(p(0)*p(0) + p(2)*p(2));
								if (d>maxLength) d = maxLength;
								const float a = atan2(p(0), p(2));
								const int32_t bin = angle2bin(a);
								if (bin>=0 and bin<LASER_SIZE and (*laserDataW)[bin].dist > d)
								{
									if ( (*laserDataW)[bin].dist > d)
										(*laserDataW)[bin].dist = d;
								}
							}
						}
					}
				}
			}
		}
		else /// If the proxy is a good old RGBD interface
		{
			RoboCompRGBD::PointSeq points;
			try
			{
				rgbds[r].proxyRGBD->getXYZ(points, hState, bState);
			}
			catch (const Ice::Exception &ex)
			{
				cout << "Can't connect to rgbd: " << ex << endl;
				continue;
			}
			RTMat TR = innerModel->getTransformationMatrix(base, QString::fromStdString(rgbds[r].id));
#ifdef STORE_POINTCLOUDS_AND_EXIT
			cloud->points.resize(points.size());
#endif
// printf("%d\n", __LINE__);

			uint32_t pw = 640;
			uint32_t ph = 480;
			uint32_t step = 13;
			if (points.size() == 320*240) { pw=320; ph=240; step=11; }
			if (points.size() == 160*120) { pw=160; ph=120; step=5; }
			if (points.size() == 80*60) { pw=80; ph=60; step=3; }
			for (uint32_t rr=0; rr<ph; rr+=step)
			{
				for (uint32_t cc=rr%5; cc<pw; cc+=2)
				{
					uint32_t ioi = rr*pw+cc;
					if (ioi<points.size())
					{
						const QVec p = (TR * QVec::vec4(points[ioi].x, points[ioi].y, points[ioi].z, 1)).fromHomogeneousCoordinates();
#ifdef STORE_POINTCLOUDS_AND_EXIT
						cloud->points[ioi].x =  p(0)/1000;
						cloud->points[ioi].y =  p(1)/1000;
						cloud->points[ioi].z = -p(2)/1000;
#endif
						if ( (p(1)>=minHeight and p(1)<=maxHeight) or (p(1)<minHeightNeg) )
						{
// 							p(1) = 0;
							float d = sqrt(p(0)*p(0) + p(2)*p(2));
							if (d>maxLength) d = maxLength;
							const float a = atan2(p(0), p(2));
							const int32_t bin = angle2bin(a);
							if (bin>=0 and bin<LASER_SIZE and (*laserDataW)[bin].dist > d)
							{
								(*laserDataW)[bin].dist = d;
							}
						}
					}
				}
			}
// printf("%d\n", __LINE__);

		}
#ifdef STORE_POINTCLOUDS_AND_EXIT
		writePCD(rgbds[r].id+".pcd", cloud);
#endif
	}
#ifdef STORE_POINTCLOUDS_AND_EXIT
	qFatal("done");
#endif

	try
	{
		RoboCompLaser::TLaserData alData = laser->getLaserData();
		for (uint i=0; i<alData.size(); i++)
		{
			if (i==alData.size()/2) printf("PC %d  (%f _ %f)\n", i, alData[i].dist, alData[i].angle);
			const QVec p = innerModel->laserTo(actualLaserID, actualLaserID, alData[i].dist, alData[i].angle);
			if (i==alData.size()/2) p.print("en base");
			if (i==alData.size()/2) printf("(%s)", base.toStdString().c_str());
			const float angle = atan2(p(0), p(2));
			const float dist = p.norm2();
			if (i==alData.size()/2) printf("enlaser %f %f\n", dist, angle);
			const int j = LASER_SIZE*angle/FOV + (LASER_SIZE/2);
			if (i==alData.size()/2) printf("index %d\n", j);
// 			printf("FOV:%f, angle:%f, LASER_SIZE=%f, j:%d\n", (float)FOV, angle, (float)LASER_SIZE, j);
			
			if (j>=0 and j<(int)laserDataW->size())
			{
				if ((*laserDataW)[j].dist > dist)
				{
					(*laserDataW)[j].dist = dist;
				}
			}
		}
	}
	catch (const Ice::Exception &ex)
	{
		cout << "Can't connect to laser: " << ex << endl;
	}

	
	
	
	
	
	
	RoboCompOmniRobot::TBaseState oState;
	try { omnirobot->getBaseState(oState); }
	catch (Ice::Exception e) { qDebug()<<"error talking to base"<<e.what(); }
	bStateOut.x     = oState.x;
	bStateOut.z     = oState.z;
	bStateOut.alpha = oState.alpha;
	// Double buffer swap
	RoboCompLaser::TLaserData *t = laserDataR;
	mutex->lock();
	laserDataR = laserDataW;
	mutex->unlock();
	innerModel->updateTransformValues("robot", bStateOut.x, 0, bStateOut.z, 0, bStateOut.alpha,0);
// 	printf("%f %f ___ %f\n", bStateOut.x, bStateOut.z, bStateOut.alpha);
#ifdef USE_EXTENSION
	extended->update(*laserDataR);
	static QTime te = QTime::currentTime();
	float re = 11.*te.elapsed()/1000.;
	te = QTime::currentTime();
// 	printf("S ------------   %d       %f\n", extended->size(), re);
	extended->relax(re, innerModel, "laser", "root");
#endif

// 	medianFilter();
	laserDataW = t;
}

/*
void Worker::writePCD(std::string path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	printf("Writing: %s  width:%d height:%d points:%d\n", path.c_str(), (int)cloud->width, (int)cloud->height, (int)cloud->points.size());
	cloud->width = 1;
	cloud->height = cloud->points.size();
	static pcl::PCDWriter writer;
	if (not cloud->empty()) writer.writeASCII(path, *cloud);
}
*/

RoboCompLaser::TLaserData Worker::getLaserData()
{
	QMutexLocker m(mutex);
#ifdef USE_EXTENSION
	return extended->getData();
#else
	return *laserDataR;
#endif
}

RoboCompLaser::TLaserData Worker::getLaserAndBStateData(RoboCompDifferentialRobot::TBaseState &baseState)
{
	QMutexLocker m(mutex);
	baseState = bStateOut;
#ifdef USE_EXTENSION
	return extended->getData();
#else
	return *laserDataR;
#endif
}

RoboCompLaser::LaserConfData Worker::getLaserConfData()
{
	return confData;
}

void Worker::medianFilter()
{
	float window[5];
	float x[laserDataR->size()];

	for (int32_t i=2; i<(int)laserDataR->size()-2; ++i)
	{
		for (int j=0; j<5; ++j)
		{
			window[j] = laserDataR->at(i - 2 + j).dist;
		}
		int j;
		for (j=0; j<3; ++j)
		{
			int min = j;
			for (int k = j + 1; k < 5; ++k)
				if (window[k] < window[min])
					min = k;
			const float temp = window[j];
			window[j] = window[min];
			window[min] = temp;
		}
		if (window[j] >= maxLength-1)
		{
			if (window[j-1] >= maxLength-1)
			{
				x[i] = window[j-2];
			}
			else
			{
				x[i] = window[j-1];
			}
		}
		else
		{
			x[i] = window[j];
		}
	}
	for (int32_t i=0; i<(int)laserDataR->size(); i++)
	{
		(*laserDataR)[i].dist = x[i];
	}

	// Endings
	(*laserDataR)[laserDataR->size()-2].dist = laserDataR->at(laserDataR->size()-3).dist;
	(*laserDataR)[laserDataR->size()-1].dist = laserDataR->at(laserDataR->size()-2).dist;
	(*laserDataR)[1].dist = laserDataR->at(2).dist;
	(*laserDataR)[0].dist = laserDataR->at(1).dist;
}



