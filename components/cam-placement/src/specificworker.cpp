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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx) : GenericWorker(tprx)
{
	innerModelViewer = NULL;
	osgView = new OsgView(this, false);
	osgGA::TrackballManipulator *tb = new osgGA::TrackballManipulator;
	osg::Vec3d eye(osg::Vec3(4000.,4000.,-1000.));
	osg::Vec3d center(osg::Vec3(0.,0.,-0.));
	osg::Vec3d up(osg::Vec3(0.,1.,0.));
	tb->setHomePosition(eye, center, up, true);
	tb->setByMatrix(osg::Matrixf::lookAt(eye,center,up));
	osgView->setCameraManipulator(tb);
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	innerModel = std::make_shared<InnerModel>("camera-world.xml"); 
	//innerModel->print();
	innerModelViewer = new InnerModelViewer (innerModel, "root", osgView->getRootGroup(), false);
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = 100;
	timer.setSingleShot(true);
	timer.start(Period);
}

void SpecificWorker::compute()
{
	moveCamera("rgbd");
	//for all cameras
		auto &&[frame, points] = getImage("rgbd");
		// extract all points blue and fill the grid using x,y to i,j transform
	//count ratio filled/not filled

	//put first camera and compute footprint
	//for all remaining cameras
		// compute all possible poses and footprints
		// discard those without solapamiento or with holes
		// chooes the one with maximum footprint
	
	// cv::imshow("Camara", frame);
	// cvWaitKey(1);

	if (innerModelViewer) 
		innerModelViewer->update();
	osgView->frame();
}

QPolygonF SpecificWorker::footprint(cv::Mat img)
{
	//filter out non blue pixels
	cv::Mat blue;
	cv::inRange(img, cv::Scalar(0, 0, 255), cv::Scalar(0, 0, 255), blue);
	//Extract the contours
	std::vector<cv::Vec4i> hierarchy;
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours( blue, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
	// analyze contours (hierarchy) and discard fragmented footprints

	// converto resulting contour to QPolygonF
	QPolygonF poly;
	return poly;
}

void SpecificWorker::moveCamera(const std::string &camera_id)
{
	const QString id = QString::fromStdString(camera_id);
	RTMat rt= innerModel->getTransformationMatrix("root", id);
	innerModelViewer->cameras[id].viewerCamera->getCameraManipulator()->setByMatrix(QMatToOSGMat4(rt));
}

std::tuple<cv::Mat, std::vector<SpecificWorker::PointXYZ>> SpecificWorker::getImage(const std::string &camera_id)
{
	if( innerModelViewer->cameras.size() == 0 )
		qFatal("No cameras in getImage");	

	innerModelViewer->cameras[QString::fromStdString(camera_id)].viewerCamera->frame();
	auto rgbd = innerModel->getNode<InnerModelRGBD>(camera_id);
	IMVCamera &cam = innerModelViewer->cameras[rgbd->id];
	const int width = cam.RGBDNode->width;
	const int height = cam.RGBDNode->height;
	const float focal = ( float ) cam.RGBDNode->focal;
	double fovy, aspectRatio, Zn, Zf;
	cam.viewerCamera->getCamera()->getProjectionMatrixAsPerspective ( fovy, aspectRatio, Zn, Zf );

	const float *d = (float *)cam.d->data();
	const unsigned char *rgb = cam.rgb->data();
	cv::Mat frame(height, width, CV_8UC3, (unsigned *)rgb);
	std::vector<PointXYZ> points( width*height );
	std::vector<float> depth( width*height );
	for ( int i=0; i<height; ++i ) 
	 	for ( int j=0; j<width; ++j ) 
		{
			const int index  = j + i*width;
			const int indexI = j + (height-1-i)*width;
			if ( d[indexI] <= 1. ) 
			 	depth[index] = ( Zn*Zf / ( Zf - d[indexI]* ( Zf-Zn ) ) );
			 else
			 	//depth[i] = NAN;
				depth[index] = NAN;   //check
			float x = ( depth[index] * ( ( float ) j- ( width/2. ) ) / focal );
			float y = depth[index] * ( (height/2.) - float(i) ) / focal;
			float z = depth[index];
			
			// compute world coordinates of the point
			QVec r = innerModel->transform("world", QVec::vec3(x,y,z), QString::fromStdString(camera_id));
			points[index].x = x;
			points[index].y = y;
			points[index].z = z;
		}
	return std::forward_as_tuple(frame, points);
}




