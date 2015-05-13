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
	osgView = new OsgView (frame);
	osgView->setCameraManipulator(new osgGA::TrackballManipulator); 	
	osgView->getCameraManipulator()->setHomePosition(osg::Vec3(0.,0.,-2.),osg::Vec3(0.,0.,4.),osg::Vec3(0.,1,0.));
	
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
		RoboCompCommonBehavior::Parameter par = params.at("Visor3D.InnerModelPath");
		//innerModel = new InnerModel(par.value);
		innerModel = new InnerModel(par.value);
	}
	catch(std::exception e) { qFatal("Error reading config params"); }
	
	imv = new InnerModelViewer (innerModel, "root", osgView->getRootGroup());
	
	timer.start(Period);

	return true;
}

void SpecificWorker::compute()
{
	try
	{
		vector<string> names;
		names.push_back("default");
		RoboCompRGBDBus::ImageMap imgs;
		rgbdbus_proxy->getImages(names, imgs);
		//Mat frame(imgs["default"].camera.colorHeight, imgs["default"].camera.colorWidth, CV_8UC3,  &(imgs["default"].colorImage)[0]);
		//imshow("3DViewer", frame);
		
		uint8_t *rgbImage = &(imgs["default"].colorImage)[0];
		imv->planesHash["back"]->updateBuffer(rgbImage, 640, 480);
	}
	catch(const Ice::Exception &e)
	{	std::cout << "Error reading from Camera" << e << std::endl;	}
	
	imv->update();
	osgView->frame();

}





