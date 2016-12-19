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
 // namedWindow( "Display window", 1 );// Create a window for display.
   
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
}

void SpecificWorker::compute()
{
    static RoboCompGenericBase::TBaseState bState;
    static RoboCompJointMotor::MotorStateMap hState;
    static RoboCompRGBD::imgType rgbMatrix;
    static RoboCompRGBD::depthType distanceMatrix;
     
    try
    {
        qDebug() << "read frame";
   
        rgbd_proxy->getData(rgbMatrix,distanceMatrix, hState, bState);
        
        Mat frame(480, 640, CV_8UC3,  &(rgbMatrix)[0]);
        imshow("3D viewer",frame);
        
        QImage img = QImage(&rgbMatrix[0], 640, 480, QImage::Format_RGB888);
        label->setPixmap(QPixmap::fromImage(img));
        label->resize(label->pixmap()->size());

    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
}


