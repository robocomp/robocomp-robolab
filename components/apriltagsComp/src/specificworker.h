/*
 *    Copyright (C) 2006-2019 by RoboLab - University of Extremadura
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

/**
       \brief
       @author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include "opencv2/opencv.hpp"

// April tags detector and various families that can be selected by command line option
#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag16h5.h>
#include <AprilTags/Tag25h7.h>
#include <AprilTags/Tag25h9.h>
#include <AprilTags/Tag36h9.h>
#include <AprilTags/Tag36h11.h>

#define PI M_PI
#define TWOPI  2.0*M_PI

#define W 640
#define H 480


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	//virtual void  newAprilTag(int id, float tx, float ty, float tz, float rx, float ry, float rz){};
	listaMarcas GetAprilTags_checkMarcas();
	RoboCompCommonBehavior::ParameterList getWorkerParams();

public slots:
	void compute();
	void initialize(int period);
private:
	::AprilTags::TagDetector* m_tagDetector;
	::AprilTags::TagCodes m_tagCodes;
	RoboCompCamera::TCamParams camParams;
	RoboCompGenericBase::TBaseState bState;
	RoboCompJointMotor::MotorStateMap hState;
	RoboCompCommonHead::THeadState cState;
	enum  {Camera, RGBD, RGBDBus, CameraSimple};  	
	QMap<int, float> tagsSizeMap;
	bool flip;
	
	vector<RoboCompAprilTags::tag> detections2send;
	vector<RoboCompGetAprilTags::marca> listaDeMarcas;
	
	bool m_draw; // draw image and April tag detections?
	int m_width; // image size in pixels
	int m_height;
	double m_tagSize; // April tag side length in meters of square black frame
	double m_fx; // camera focal length in pixels
	double m_fy;
	double m_px; // camera principal point
	double m_py;
	string camera_name;
	string innermodel_path;
	std::shared_ptr<InnerModel> innerModel;
	
	void print_detection(vector< ::AprilTags::TagDetection> detections);
	void loop();
	void setTagCodes(std::string s);
	double tic();
	
	/**
	* Normalize angle to be within the interval [-pi,pi].
	*/
	inline double standardRad(double t);
	void rotationFromMatrix(const Eigen::Matrix3d &R, double &rx, double &ry, double &rz);
	void searchTags(const cv::Mat &image_gray);
	cv::Mat image_gray, image_color;
	int INPUTIFACE;
	
	RoboCompCommonBehavior::ParameterList worker_params;
	QMutex *worker_params_mutex;
};

#endif
