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
#include <csignal>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx)
		:GenericWorker(mprx)
{
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	qDebug() << "Destroying SpecificWorker";
	this->terminate();
}

void SpecificWorker::terminate()
{
	std::cout << "Terminating astra" << std::endl;
	astra::terminate();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{

	depthB = QString::fromStdString(params["depth"].value).contains("true");
	colorB = QString::fromStdString(params["color"].value).contains("true");
	bodyB = QString::fromStdString(params["body"].value).contains("true");
	pointB = QString::fromStdString(params["point"].value).contains("true");
        device = params["device"].value;
        cameraID = QString::fromStdString(params["cameraID"].value).toFloat();
        astra::initialize();
        
        astra::StreamSet streamSet(device.c_str());

	frameListener = new MultiFrameListener(this->humantrackerjointsandrgb_pubproxy, cameraID);
	frameListener->set_color_stream(colorB);
	qDebug() << "Color stream will be opened? " << colorB;
	frameListener->set_depth_stream(depthB);
	qDebug() << "Depth stream will be opened? " << depthB;
	frameListener->set_point_stream(pointB);
	qDebug() << "Points  stream will be opened? " << pointB;
	frameListener->set_body_stream(bodyB);
	qDebug() << "Body stream will be opened? " << bodyB;

//	timer.start(Period);
//    initializeStreams();


	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	//currently there's no default way to change the period without getting this overwrited by initialize
	this->Period = 1/30;
	timer.start(Period);
}

void SpecificWorker::compute()
{
	QMutexLocker locker(mutex);
	astra_update();

}

float SpecificWorker::compute_fps(bool print)
{
	typedef std::chrono::duration<float> fsec;
	std::chrono::system_clock::time_point current_time = std::chrono::system_clock::now();
	fsec delta_time = current_time-last_time;
	if (print) {
		std::cout << fsec(1)/delta_time << endl;
	}
	last_time = current_time;
	return fsec(1)/delta_time;
}

Registration SpecificWorker::RGBD_getRegistration()
{
	qDebug() << "getRGBDParams Not implemented yet";
}

void
SpecificWorker::RGBD_getData(imgType& rgbMatrix, depthType& distanceMatrix, RoboCompJointMotor::MotorStateMap& hState,
		RoboCompGenericBase::TBaseState& bState)
{
	if (!this->colorB) {
		std::cout << "WARNING: A request for a not initiated STREAM have been received." << endl;
		std::cout
				<< "WARNING: Probably you want to check the \"color\" flag in etc/config or the config file you are using."
				<< endl;
		return;
	}
	if (!this->depthB) {
		std::cout << "WARNING: A request for a not initiated STREAM have been received." << endl;
		std::cout
				<< "WARNING: Probably you want to check the \"depth\" flag in etc/config or the config file you are using."
				<< endl;
		return;
	}
	frameListener->get_color(rgbMatrix);
	frameListener->get_depth(distanceMatrix);

}

void
SpecificWorker::RGBD_getXYZ(PointSeq& points, RoboCompJointMotor::MotorStateMap& hState,
		RoboCompGenericBase::TBaseState& bState)
{
	typedef std::chrono::duration<float> fsec;
	std::chrono::system_clock::time_point initial_time = std::chrono::system_clock::now();

	if (!this->pointB) {
		std::cout << "WARNING: A request for a not initiated STREAM have been received." << endl;
		std::cout
				<< "WARNING: Probably you want to check the \"points\" flag in etc/config or the config file you are using."
				<< endl;
		return;
	}
	frameListener->get_points(points);

	std::cout << "RGBD_getXYZ: fps " << fsec(1)/(std::chrono::system_clock::now()-initial_time) << endl;
	compute_fps(true);

}

void
SpecificWorker::RGBD_getRGB(ColorSeq& color, RoboCompJointMotor::MotorStateMap& hState,
		RoboCompGenericBase::TBaseState& bState)
{
	if (!this->colorB) {
		std::cout << "WARNING: A request for a not initiated STREAM have been received." << endl;
		std::cout
				<< "WARNING: Probably you want to check the \"color\" flag in etc/config or the config file you are using."
				<< endl;
		return;
	}
	frameListener->get_color(color);
}

TRGBDParams SpecificWorker::RGBD_getRGBDParams()
{
	qDebug() << "getRGBDParams Not implemented yet";
}

void
SpecificWorker::RGBD_getDepth(DepthSeq& depth, RoboCompJointMotor::MotorStateMap& hState,
		RoboCompGenericBase::TBaseState& bState)
{
//implementCODE
	if (!this->depthB) {
		std::cout << "WARNING: A request for a not initiated STREAM have been received." << endl;
		std::cout
				<< "WARNING: Probably you want to check the \"depth\" flag in etc/config or the config file you are using."
				<< endl;
		return;
	}
	frameListener->get_depth(depth);
}

void SpecificWorker::RGBD_setRegistration(const Registration& value)
{
//implementCODE
	qDebug() << "RGBD_setRegistration Not implemented yet";
}

void
SpecificWorker::RGBD_getXYZByteStream(imgType& pointStream, RoboCompJointMotor::MotorStateMap& hState,
		RoboCompGenericBase::TBaseState& bState)
{
//implementCODE
	if (!this->pointB) {
		std::cout << "WARNING: A request for a not initiated STREAM have been received." << endl;
		std::cout
				<< "WARNING: Probably you want to check the \"points\" flag in etc/config or the config file you are using."
				<< endl;
		return;
	}
	frameListener->get_points_stream(pointStream);

}

void
SpecificWorker::RGBD_getImage(ColorSeq& color, DepthSeq& depth, PointSeq& points,
		RoboCompJointMotor::MotorStateMap& hState, RoboCompGenericBase::TBaseState& bState)
{
//implementCODE
	if (!this->colorB) {
		std::cout << "WARNING: A request for a not initiated STREAM have been received." << endl;
		std::cout
				<< "WARNING: Probably you want to check the \"color\" flag in etc/config or the config file you are using."
				<< endl;
		return;
	}
	if (!this->depthB) {
		std::cout << "WARNING: A request for a not initiated STREAM have been received." << endl;
		std::cout
				<< "WARNING: Probably you want to check the \"depth\" flag in etc/config or the config file you are using."
				<< endl;
		return;
	}
	if (!this->pointB) {
		std::cout << "WARNING: A request for a not initiated STREAM have been received." << endl;
		std::cout
				<< "WARNING: Probably you want to check the \"points\" flag in etc/config or the config file you are using."
				<< endl;
		return;
	}

	frameListener->get_color(color);
	frameListener->get_depth(depth);
	frameListener->get_points(points);

}

void
SpecificWorker::RGBD_getDepthInIR(depthType& distanceMatrix, RoboCompJointMotor::MotorStateMap& hState,
		RoboCompGenericBase::TBaseState& bState)
{
	qDebug() << "getDepthInIR Not implemented yet";
}

void SpecificWorker::HumanTracker_getJointsPosition(const int id, RoboCompHumanTracker::jointListType& jointList)
{
//implementCODE
	qDebug() << "HumanTracker_getJointsPosition Not implemented yet";
}

void SpecificWorker::HumanTracker_getRTMatrixList(const int id, RoboCompHumanTracker::RTMatrixList& RTMatList)
{
//implementCODE
	qDebug() << "HumanTracker_getRTMatrixList Not implemented yet";
}

void SpecificWorker::HumanTracker_getUser(const int id, RoboCompHumanTracker::TPerson& user)
{
//implementCODE
	qDebug() << "HumanTracker_getUser Not implemented yet";
}

bool SpecificWorker::HumanTracker_getJointDepthPosition(const int idperson, const string& idjoint, RoboCompHumanTracker::joint& depthjoint)
{

	if (!this->bodyB) {
		std::cout << "WARNING: A request for a not initiated STREAM have been received." << endl;
		std::cout
				<< "WARNING: Probably you want to check the \"body\" flag in etc/config or the config file you are using."
				<< endl;
		return false;
	}
	depthjoint = frameListener->getJointDepth(idperson, idjoint);

	if (depthjoint.size()>0)
		return true;

	else {
		depthjoint = {};
		return false;
	};
}

void SpecificWorker::HumanTracker_getUsersList(RoboCompHumanTracker::PersonList& users)
{
	if (!this->bodyB) {
		std::cout << "WARNING: A request for a not initiated STREAM have been received." << endl;
		std::cout
				<< "WARNING: Probably you want to check the \"body\" flag in etc/config or the config file you are using."
				<< endl;
		return;
	}
	frameListener->get_people(users);
}

void SpecificWorker::HumanTracker_getUserState(const int id, RoboCompHumanTracker::TrackingState& state)
{
//implementCODE
	qDebug() << "HumanTracker_getUserState Not implemented yet";
}
