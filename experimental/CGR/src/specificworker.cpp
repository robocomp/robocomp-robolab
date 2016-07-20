//
//    Copyright (C) 2015 by YOUR NAME HERE
//
//    This file is part of RoboComp
//
//    RoboComp is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    RoboComp is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
//
// based in Joydeep Biswas, (C) 2010 paper
//
//========================================================================
//


#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <string.h>
#include "proghelp.h"

#include "vectorparticlefilter.h"
#include "vector_map.h"
#include "terminal_utils.h"
#include "timer.h"



#include "configreader.h"
//#include "plane_filtering.h"

#include "specificworker.h"

bool run = true;
bool resetCgr=false;
bool usePointCloud = false;
bool noLidar = false;
int numParticles = 20;
int debugLevel = -1;

int cont=0; //to calculate fps

RoboCompOmniRobot::TBaseState bStateOld;

using namespace std;
/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	t.start();
	innerModelViewer = NULL;
	osgView = new OsgView (widget);
	osgGA::TrackballManipulator *tb = new osgGA::TrackballManipulator;
	osg::Vec3d eye(osg::Vec3(000.,3000.,-6000.));
	osg::Vec3d center(osg::Vec3(0.,0.,-0.));
	osg::Vec3d up(osg::Vec3(0.,1.,0.));
	tb->setHomePosition(eye, center, up, true);
	tb->setByMatrix(osg::Matrixf::lookAt(eye,center,up));
	osgView->setCameraManipulator(tb);
// 	omnirobot_proxy->getBaseState(bStateOld);
}
/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
}



void SpecificWorker::LoadParameters()
{
  WatchFiles watch_files;
  ConfigReader config("/home/robocomp/robocomp/components/robocomp-robolab/experimental/CGR/etc/"); //path a los ficheros de configuracion desde el path del binario.
  config.init(watch_files);
  
  config.addFile("localization_parameters.cfg");
  config.addFile("kinect_parameters.cfg");
  if(!config.readFiles())
  {
    printf("Failed to read config\n");
    exit(1);
  }
/**
    {
    ConfigReader::SubTree c(config,"KinectParameters");
    
    unsigned int maxDepthVal;
    bool error = false;
    error = error || !c.getReal("f",kinectDepthCam.f);
    error = error || !c.getReal("fovH",kinectDepthCam.fovH);
    error = error || !c.getReal("fovV",kinectDepthCam.fovV);
    error = error || !c.getInt("width",kinectDepthCam.width);
    error = error || !c.getInt("height",kinectDepthCam.height);
    error = error || !c.getUInt("maxDepthVal",maxDepthVal);
    kinectDepthCam.maxDepthVal = maxDepthVal;    
    
    vector3f kinectLoc;
    float xRot, yRot, zRot;
    error = error || !c.getVec3f("loc",kinectLoc);
    error = error || !c.getReal("xRot",xRot);
    error = error || !c.getReal("yRot",yRot);
    error = error || !c.getReal("zRot",zRot);
    kinectToRobotTransform.xyzRotationAndTransformation(xRot,yRot,zRot,kinectLoc);
    
    if(error){
      printf("Error Loading Kinect Parameters!\n");
      exit(2);
    }
    }

    {
    ConfigReader::SubTree c(config,"PlaneFilteringParameters");
    
    bool error = false;
    error = error || !c.getUInt("maxPoints",filterParams.maxPoints);
    error = error || !c.getUInt("numSamples",filterParams.numSamples);
    error = error || !c.getUInt("numLocalSamples",filterParams.numLocalSamples);
    error = error || !c.getUInt("planeSize",filterParams.planeSize);
    error = error || !c.getReal("maxError",filterParams.maxError);
    error = error || !c.getReal("maxDepthDiff",filterParams.maxDepthDiff);
    error = error || !c.getReal("minInlierFraction",filterParams.minInlierFraction);
    error = error || !c.getReal("WorldPlaneSize",filterParams.WorldPlaneSize);
    error = error || !c.getUInt("numRetries",filterParams.numRetries);
    filterParams.runPolygonization = false;
    
    if(error){
      printf("Error Loading Plane Filtering Parameters!\n");
      exit(2);
    }
  }
*/ 
  
  {
    ConfigReader::SubTree c(config,"initialConditions");
    bool error = false;
    curMapName = string(c.getStr("mapName"));
    error = error || curMapName.length()==0;
    error = error || !c.getVec2f("loc",initialLoc);
    error = error || !c.getReal("angle", initialAngle);
    error = error || !c.getReal("locUncertainty", locUncertainty);
    error = error || !c.getReal("angleUncertainty", angleUncertainty);
    
    if(error){
      printf("Error Loading Initial Conditions!\n");
      exit(2);
    }
  }
  
  {
    ConfigReader::SubTree c(config,"motionParams");
    
    bool error = false;
    error = error || !c.getReal("Alpha1", motionParams.Alpha1);
    error = error || !c.getReal("Alpha2", motionParams.Alpha2);
    error = error || !c.getReal("Alpha3", motionParams.Alpha3);
    error = error || !c.getReal("kernelSize", motionParams.kernelSize);
    
    
    if(error){
      printf("Error Loading Predict Parameters!\n");
      exit(2);
    }
  }
  
  {
    ConfigReader::SubTree c(config,"lidarParams");
    
    bool error = false;
    // Laser sensor properties  //BASURA
    error = error || !c.getReal("angleResolution", lidarParams.angleResolution);
    error = error || !c.getReal("minAngle", lidarParams.minAngle);
    error = error || !c.getReal("maxAngle", lidarParams.maxAngle);
    error = error || !c.getInt("numRays", lidarParams.numRays);
    error = error || !c.getReal("maxRange", lidarParams.maxRange);
    error = error || !c.getReal("minRange", lidarParams.minRange);
    
    // Pose of laser sensor on robot	//BASURA
    vector2f laserToBaseTrans;
    float xRot, yRot, zRot;
    error = error || !c.getVec2f<vector2f>("laserLoc", laserToBaseTrans);
    error = error || !c.getReal("xRot", xRot);
    error = error || !c.getReal("yRot", yRot);
    error = error || !c.getReal("zRot", zRot);
    Matrix3f laserToBaseRot;
    laserToBaseRot = AngleAxisf(xRot, Vector3f::UnitX()) * AngleAxisf(yRot, Vector3f::UnitY()) * AngleAxisf(zRot, Vector3f::UnitZ());
    lidarParams.laserToBaseTrans = Vector2f(V2COMP(laserToBaseTrans));
    lidarParams.laserToBaseRot = laserToBaseRot.block(0,0,2,2);
    
    // Parameters related to observation update
    error = error || !c.getReal("logObstacleProb", lidarParams.logObstacleProb);
    error = error || !c.getReal("logOutOfRangeProb", lidarParams.logOutOfRangeProb);
    error = error || !c.getReal("logShortHitProb", lidarParams.logShortHitProb);
    error = error || !c.getReal("correlationFactor", lidarParams.correlationFactor);
    error = error || !c.getReal("lidarStdDev", lidarParams.lidarStdDev);
    error = error || !c.getReal("attractorRange", lidarParams.attractorRange);
    error = error || !c.getReal("kernelSize", lidarParams.kernelSize);
    
    // Parameters related to observation refine
    error = error || !c.getInt("minPoints", lidarParams.minPoints);
    error = error || !c.getInt("numSteps", lidarParams.numSteps);
    error = error || !c.getReal("etaAngle", lidarParams.etaAngle);
    error = error || !c.getReal("etaLoc", lidarParams.etaLoc);
    error = error || !c.getReal("maxAngleGradient", lidarParams.maxAngleGradient);
    error = error || !c.getReal("maxLocGradient", lidarParams.maxLocGradient);
    error = error || !c.getReal("minCosAngleError", lidarParams.minCosAngleError);
    error = error || !c.getReal("correspondenceMargin", lidarParams.correspondenceMargin);
    error = error || !c.getReal("minRefineFraction", lidarParams.minRefineFraction);
    
    lidarParams.initialize();
    
    if(error){
      printf("Error Loading Lidar Parameters!\n");
      exit(2);
    }
  }


}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	//Inintializing parameters for CGR
	LoadParameters();

	//Inintializing InnerModel with ursus.xml
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		
		qDebug() << QString::fromStdString(par.value);
		if( QFile::exists(QString::fromStdString(par.value)) )
		{
			innerModel = new InnerModel(par.value);
			innerModelViewer = new InnerModelViewer(innerModel, "root", osgView->getRootGroup(), true);
		}
		else
		{
			std::cout << "Innermodel path " << par.value << " not found. "; qFatal("Abort");
		}
        }
        catch(std::exception e)
	{
		qFatal("Error reading config params");
	}
	
	printf("NumParticles     : %d\n",numParticles);
	printf("Alpha1           : %f\n",motionParams.Alpha1);
	printf("Alpha2           : %f\n",motionParams.Alpha2);
	printf("Alpha3           : %f\n",motionParams.Alpha3);
	printf("UsePointCloud    : %d\n",usePointCloud?1:0);
	printf("UseLIDAR         : %d\n",noLidar?0:1);
	printf("Visualizations   : %d\n",debugLevel>=0?1:0);
	printf("\n");
  
	//double seed = floor(fmod(GetTimeSec()*1000000.0,1000000.0));
	//if(debugLevel>-1) printf("Seeding with %d\n",(unsigned int)seed);
	//srand(seed);

	InnerModelDraw::addTransform(innerModelViewer,"poseRob1","floor");
	InnerModelDraw::addTransform(innerModelViewer,"poseRob2","floor");

	//Una vez cargado el innermodel y los parametros, cargamos los mapas con sus lineas y las pintamos.

	string mapsFolder("etc/maps");
	localization = new VectorLocalization2D(mapsFolder.c_str());
	localization->initialize(numParticles,
	curMapName.c_str(),initialLoc,initialAngle,locUncertainty,angleUncertainty);

	qDebug()<<"<<<<<<<<<<<<<<< setparams1 >>>>>>>>>>>";	
	drawLines();    
	qDebug()<<"<<<<<<<<<<<<<<< setparams2 >>>>>>>>>>>";	
	drawParticles();
	qDebug()<<"<<<<<<<<<<<<<<< setparams3 >>>>>>>>>>>";	
	timer.start(Period);
	qDebug()<<"<<<<<<<<<<<<<<< setparamsF >>>>>>>>>>>";
	return true;
}



void SpecificWorker::compute()
{
	static QTime fp = QTime::currentTime();
	if (fp.elapsed() < 300)
	{
		return;
	}
	fp = QTime::currentTime();
 
	qDebug()<<"<<<<<<<<<<<<<<< compute >>>>>>>>>>>";
	bool forcePredict = false;
	mutex->lock();
	if (resetCgr)
	{
		  qDebug()<< "reset base" << curLoc.x << curLoc.y << curAngle;
			for (uint i=0; i<localization->particles.size(); i++)
			{
					localization->particles[i].lastLoc.x=localization->particles[i].loc.x=curLoc.x;
					localization->particles[i].lastLoc.y=localization->particles[i].loc.y=curLoc.y;
					localization->particles[i].angle=curAngle;
			}
			resetCgr = false;
			forcePredict = true;
	}
	RoboCompOmniRobot::TBaseState bState;
	omnirobot_proxy->getBaseState(bState);
	innerModel->updateTransformValues("poseRob1", bStateOld.x, 0, bStateOld.z, 0, bStateOld.alpha, 0);
	innerModel->updateTransformValues("poseRob2", bState.x,    0,    bState.z, 0,    bState.alpha, 0);
	auto diff = innerModel->transform6D("poseRob1", "poseRob2");
	if(fabs(diff(2)) > 10 or (fabs(diff(0)) > 10) or fabs(diff(4)) > 0.01 or forcePredict)
	{
		localization->predict(diff(2)/1000.f,-diff(0)/1000.f , -diff(4), motionParams);
	}
	bStateOld = bState;
	innerModel->updateTransformValues("robot", bState.x, 0, bState.z, 0, bState.alpha, 0);
	updateLaser();
	
	localization->refineLidar(lidarParams);
	localization->updateLidar(lidarParams, motionParams);
	localization->resample(VectorLocalization2D::LowVarianceResampling);
	localization->computeLocation(curLoc,curAngle);
// 	if(fabs(bStateOld.correctedX - (-curLoc.y*1000)) > 10 or (fabs(bStateOld.correctedZ - curLoc.x*1000)) > 10 or fabs(bStateOld.correctedAlpha - (-curAngle)) > 0.03)
	float poseCertainty = cgrCertainty();
// 	if(poseUncertainty>0.4)
	{	
		printf("Certainty: %f, curloc (%f,%f,%f)\n",poseCertainty,-curLoc.y*1000,curLoc.x*1000,-curAngle);
		cgrtopic_proxy->newCGRPose(poseCertainty,-curLoc.y*1000, curLoc.x*1000, -curAngle);
	}
	updateParticles();

	innerModelViewer->update();
 	osgView->autoResize();
 	osgView->frame();
	mutex->unlock();
	
	if(t.elapsed()>1000)
	{	
		qDebug()<<"fps"<<cont;
		t.restart();
		cont=0;
	}
	cont++;
}

void SpecificWorker::filterParticle()
{
/*
  //Call particle filter
    vector<vector2f> pointCloud2D, pointCloudNormals2D;

    for(unsigned int i=0; i<filteredPointCloud.size(); i++){
      if(fabs(pointCloudNormals[i].z)>sin(RAD(30.0)))
        continue;

      vector2f curNormal(V2COMP(pointCloudNormals[i]));
      vector2f curPoint(V2COMP(filteredPointCloud[i]));
      curNormal.normalize();
      pointCloudNormals2D.push_back(curNormal);
      pointCloud2D.push_back(curPoint);
    }

    double start = GetTimeSec();
    localization->refinePointCloud(pointCloud2D, pointCloudNormals2D, pointCloudParams);
    localization->updatePointCloud(pointCloud2D, pointCloudNormals2D, motionParams, pointCloudParams);
    localization->resample(VectorLocalization2D::SparseMultinomialResampling);
    localization->computeLocation(curLoc,curAngle);
    std::cerr << "point cloud update: " << GetTimeSec()-start << std::endl;
*/
}

void SpecificWorker::drawLines()
{
	float p0x;
	float p0y;
	float p1x;
	float p1y;
	vector<VectorMap> maps = localization->getMaps();
	int i = 0;
	for( auto m : maps)
	{
		for( auto l: m.lines)
		{
                        p0x =l.p0.x * 1000.f;
                        p0y =l.p0.y * 1000.f;
                        p1x =l.p1.x * 1000.f;
                        p1y =l.p1.y * 1000.f;
                        
			QVec n = QVec::vec2(p1y-p0y,p1x-p0x);
			float width = (QVec::vec2(p1x-p0x,p1y-p0y)).norm2();
                        std::ostringstream oss;
                        oss << m.mapName << i;                        
			InnerModelDraw::addPlane_notExisting(
			  innerModelViewer,
			  QString::fromStdString("LINEA_"+oss.str()), "floor",
			  QVec::vec3(-(p0y+p1y)/2,0,(p0x+p1x)/2),
			  QVec::vec3(n(1),0,n(0)),"#00A0A0",
			  QVec::vec3(width, 5, 5)
			);	
			printf("tx: %f| tz: %f| nx: %f| nz: %f| width %f\n",-(p0y+p1y)/2,(p0x+p1x)/2,-n(1),n(0),width);
			i++;                        
		}
	}
}
void SpecificWorker::drawParticles()
{
	for(uint i = 0; i<localization->particles.size(); ++i)
	{
		const QString transf = QString::fromStdString("particle_")+QString::number(i);
		const QString item = QString::fromStdString("plane_")+QString::number(i);
		const QString item2= QString::fromStdString("orientacion_")+QString::number(i);
		InnerModelDraw::addTransform(innerModelViewer,transf,"floor");
		InnerModelDraw::addPlane_notExisting(innerModelViewer, item,transf,QVec::vec3(0,0,0),QVec::vec3(1,0,0),"#0000AA",QVec::vec3(100, 50, 100));
		InnerModelDraw::addPlane_notExisting(innerModelViewer, item2,transf,QVec::vec3(0,0,100),QVec::vec3(1,0,0),"#FF00AA",QVec::vec3(200, 20, 20));
	}
	InnerModelDraw::addTransform(innerModelViewer,"redTransform","floor");
	InnerModelDraw::addPlane_notExisting(innerModelViewer,"red","redTransform",QVec::vec3(0,0,0),QVec::vec3(1,0,0),"#AA0000",QVec::vec3(200, 1000, 200));
	InnerModelDraw::addPlane_notExisting(innerModelViewer, "orientacion","redTransform",QVec::vec3(0,500,200),QVec::vec3(1,0,0),"#00FF00",QVec::vec3(200, 50, 50));

	//Draw Laser
	int contador=0;
	RoboCompLaser::TLaserData laserData;
	laserData = laser_proxy->getLaserData();
        for(uint i=0;i<laserData.size();i++)
        {
// 		if(contador>=100 and contador<=668)
		{
			const QString item = QString::fromStdString("laserPoint_")+QString::number(i);
			const QString transf = QString::fromStdString("laserPointTransf_")+QString::number(i);
			InnerModelDraw::addTransform(innerModelViewer,transf,"redTransform");
			InnerModelDraw::addPlane_notExisting(innerModelViewer, item,transf,QVec::vec3(0,0,0),QVec::vec3(0,1,0),"#FFFFFF",QVec::vec3(50, 50, 50));
		}
		contador++;
	}
}


void SpecificWorker::updateParticles()
{
	int i = 0;
	for( auto particle : localization->particles)
	{
		const QString cadena = QString::fromStdString("particle_")+QString::number(i);
		if (innerModelViewer->innerModel->getNode(cadena))
		{
		        innerModel->updateTransformValues(cadena, -particle.loc.y*1000, 0, particle.loc.x*1000, 0, -particle.angle , 0, "floor");
		}
		i++;
	}
	innerModel->updateTransformValues("redTransform", -curLoc.y*1000, 0, curLoc.x*1000, 0, -curAngle, 0, "floor");
}



float SpecificWorker::cgrCertainty()
{
	float distTotal = 0.0;
	for( auto particle : localization->particles)
	{
		distTotal += sqrt(pow(curLoc.x - particle.loc.x,2) + pow(curLoc.y - particle.loc.y,2));
	}
	float avgDist = distTotal / numParticles;

	return (avgDist / motionParams.kernelSize);
}






////////////////////////////
///IMPLEMENTS METHODS
////////////////////////////

void SpecificWorker::resetPose(const float x, const float z, const float alpha)
{
	printf("me llega un resetpose\n");
	mutex->lock();

	curLoc.x = z/1000;
	curLoc.y = -x/1000;
	curAngle = -alpha;
	resetCgr=true;
	mutex->unlock();
}

////////////////////////////
///  SERVANTS
////////////////////////////

void SpecificWorker::newFilteredPoints(const OrientedPoints &ops)
{
//	for (int i = 0; i < ops.size(); i++)
//	{
//		cout << "Points: "<<ops[i].x<<" "<< ops[i].y<<" "<< ops[i].z<<endl;
//		cout << "Normals: "<<ops[i].nx<<" "<< ops[i].ny<<" "<< ops[i].nz<<endl;
//	}
}

void SpecificWorker::updateLaser()
{
    RoboCompLaser::TLaserData laserData;
    laserData = laser_proxy->getLaserData();
	
    //if(int(laserData.size()) != lidarParams.numRays){
    if(int(laserData.size()) != lidarParams.numRays){
        printf("Incorrect number of Laser Scan rays!\n");
        printf("received: %d\n",int(laserData.size()));
    }
    else
    {
        int j=0, cont=0;
        for(auto i : laserData)
        {
// 		if(cont>=100 and cont<=668)
		{
			const QString transf = QString::fromStdString("laserPointTransf_")+QString::number(cont);
			lidarParams.laserScan[j] = i.dist/1000.f;
			innerModel->updateTransformValues(transf, i.dist*sin(i.angle), 0, i.dist*cos(i.angle), 0, 0, 0, "redTransform");
			j++;
		}
		cont++;
	}
    }
}
