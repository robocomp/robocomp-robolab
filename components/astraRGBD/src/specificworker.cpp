/*
 *    Copyright (C)2018 by YOUR NAME HERE
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
#include <csignal>


/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
    qDebug()<<"Destroying SpecificWorker";
    this->terminate();
}

void SpecificWorker::terminate()
{
    std::cout<<"Terminating astra"<<std::endl;
    astra::terminate();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{

    depthB = QString::fromStdString(params["depth"].value).contains("true");
	colorB = QString::fromStdString(params["color"].value).contains("true");
	bodyB = QString::fromStdString(params["body"].value).contains("true");
	pointB  = QString::fromStdString(params["point"].value).contains("true");

    astra::initialize();

// Para abrir varias camaras con streamSet("")
//    astra::StreamSet streamSet1("device/sensor0");
//    astra::StreamSet streamSet2("device/sensor1");
//    streamSet = streamSet2;


    reader = new astra::StreamReader(streamSet.create_reader());
    frameListener = new MultiFrameListener(*reader);
//	timer.start(Period);
//    initializeStreams();
    frameListener->set_color_stream(colorB);
    qDebug()<<"Color stream will be opened? "<<colorB;
    frameListener->set_depth_stream(depthB);
    qDebug()<<"Depth stream will be opened? "<<depthB;
    frameListener->set_point_stream(pointB);
    qDebug()<<"Points  stream will be opened? "<<pointB;
    frameListener->set_body_stream(bodyB);
    qDebug()<<"Body stream will be opened? "<<bodyB;

    reader->add_listener(*frameListener);
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	timer.start(Period);
}

void SpecificWorker::compute()
{
	QMutexLocker locker(mutex);
    astra_update();

}


Registration SpecificWorker::RGBD_getRegistration()
{
//implementCODE

}

void SpecificWorker::RGBD_getData(imgType &rgbMatrix, depthType &distanceMatrix, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState &bState)
{
//implementCODE
//	RGBMutex->lock();
//	rgbMatrix=*colorImage;
//	RGBMutex->unlock();
//	depthMutex->lock();
//	distanceMatrix=*depthImage;
//	depthMutex->unlock();
//    qDebug()<<"Trying to get data";

    frameListener->get_color(rgbMatrix);
    frameListener->get_depth(distanceMatrix);
//    qDebug()<<"getDepth"<<distanceMatrix.size();

}

void SpecificWorker::RGBD_getXYZ(PointSeq &points, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState &bState)
{
//implementCODE
    frameListener->get_points(points);
    if(!pointB)
    {
        std::cout<<"WARNING: point stream is deactivated by config file.";
    }
}

void SpecificWorker::RGBD_getRGB(ColorSeq &color, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState &bState)
{
//implementCODE
    frameListener->get_color(color);
    if(!colorB)
    {
        std::cout<<"WARNING: color stream is deactivated by config file.";
    }
}

TRGBDParams SpecificWorker::RGBD_getRGBDParams()
{
//implementCODE
    qDebug()<<"getRGBDParams Not implemented yet";
}

void SpecificWorker::RGBD_getDepth(DepthSeq &depth, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState &bState)
{
//implementCODE
    frameListener->get_depth(depth);
    if(!depthB)
    {
        std::cout<<"WARNING: depth stream is deactivated by config file.";
    }
}

void SpecificWorker::RGBD_setRegistration(const Registration &value)
{
//implementCODE
    qDebug()<<"setRegistration Not implemented yet";
}

void SpecificWorker::RGBD_getImage(ColorSeq &color, DepthSeq &depth, PointSeq &points, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState &bState)
{
//implementCODE
    frameListener->get_color(color);
    frameListener->get_depth(depth);
    frameListener->get_points(points);
    if(!depthB)
    {
        std::cout<<"WARNING: depth stream is deactivated by config file.";
    }
    if(!colorB)
    {
        std::cout<<"WARNING: color stream is deactivated by config file.";
    }
    if(!pointB)
    {
        std::cout<<"WARNING: point stream is deactivated by config file.";
    }
}

void SpecificWorker::RGBD_getDepthInIR(depthType &distanceMatrix, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState &bState)
{
    qDebug()<<"getDepthInIR Not implemented yet";
}

void  SpecificWorker::HumanTracker_getUsersList(PersonList &users){
    frameListener->get_people(users);

};

bool SpecificWorker::HumanTracker_getJointDepthPosition(const int idperson, const string &idjoint, joint &depthjoint)
{
    depthjoint = frameListener->getJointDepth(idperson, idjoint);

    if (depthjoint.size()>0)
        return true;

    else
    {
        depthjoint = {};
        return false;
    };


}




