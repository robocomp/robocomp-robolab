/*
 *    Copyright (C) 2015 by YOUR NAME HERE
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

#ifdef USE_QTGUI
	innerViewer = NULL;
	osgView = new OsgView(this);
	osgGA::TrackballManipulator *tb = new osgGA::TrackballManipulator;
	osg::Vec3d eye(osg::Vec3(4000.,4000.,-1000.));
	osg::Vec3d center(osg::Vec3(0.,0.,-0.));
	osg::Vec3d up(osg::Vec3(0.,1.,0.));
	tb->setHomePosition(eye, center, up, true);
	tb->setByMatrix(osg::Matrixf::lookAt(eye,center,up));
 	osgView->setCameraManipulator(tb);
#endif
	hide();
	
	mrpt::utils::CConfigFileMemory configDifodo(default_cfg_txt);
	odo.loadConfiguration( configDifodo, rgbd_proxy );
	odo.reset();
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("visualodometryDIFODO.InnerModel");
		if( QFile::exists(QString::fromStdString(par.value)) )
		{
			innerModel = new InnerModel(par.value);
#ifdef USE_QTGUI
			innerViewer = new InnerModelViewer(innerModel, "root", osgView->getRootGroup(), true);
			show();
#endif
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
//       THE FOLLOWING IS JUST AN EXAMPLE
//
// 	try
// 	{
// 		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
// 		innermodel_path=par.value;
// 		innermodel = new InnerModel(innermodel_path);
// 	}
// 	catch(std::exception e) { qFatal("Error reading config params"); }
	
	
	timer.start(Period);

	return true;
}

void SpecificWorker::compute()
{
	try
	{
		odo.loadFrame( );
		odo.odometryCalculation();
		cout << "pose" << odo.cam_pose << endl;
		mrpt::math::CArrayDouble<6> v;
		odo.cam_pose.getAsVector( v );
		innerModel->updateTransformValues("xtion", v[0]*1000, v[1]*1000, v[2]*1000, v[3], v[4], v[5]);
	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading from Camera" << e << std::endl;
	}
	
	#ifdef USE_QTGUI
	if (innerViewer)
	{
		//QMutexLocker ml(&mutex_inner);
		innerViewer->update();
		osgView->autoResize();
		osgView->frame();
	}
	#endif
}





