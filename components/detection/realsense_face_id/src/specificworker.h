/*
 *    Copyright (C) 2021 by Alejandro Torrejon Harto
 *    Date: 26/07/2021
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

#include "RealSenseID/Preview.h"
#include "RealSenseID/FaceAuthenticator.h"
#include <iostream>
#include <stdio.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp> 
#include <opencv2/imgproc.hpp>

///////////////////////////////IMAGE////////////////////////////
class PreviewRender : public RealSenseID::PreviewImageReadyCallback 
{ 
public: 
    RealSenseID::Image frame; 
    std::mutex *mutex;
    PreviewRender(std::mutex *mutex){ 
        this->mutex=mutex; 
    } 
    void OnPreviewImageReady(const RealSenseID::Image image){ 
        //std::cout << "frame #" << image.number << ": " << image.width << "x" << image.height << " (" << image.size << " bytes)" << "stride: "<< image.stride << std::endl; 
        mutex->lock(); 
            this->frame=image; 
        mutex->unlock(); 
    } 
}; 

//////////////////SPECIFICWORKER///////////////////////////

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	RoboCompCameraSimple::TImage CameraSimple_getImage();
	RoboCompRealSenseFaceID::UserDataList RealSenseFaceID_authenticate();
	bool RealSenseFaceID_enroll(std::string user);
	bool RealSenseFaceID_eraseAll();
	bool RealSenseFaceID_eraseUser(std::string user);
	RoboCompRealSenseFaceID::UserDataList RealSenseFaceID_getQueryUsers();
	bool RealSenseFaceID_startPreview();
	bool RealSenseFaceID_stopPreview();
    vector<vector<cv::Point2i>>  face_bounding_box_spec;

//    int top_l, top_r, bottom_l, bottom_r;

public slots:
	void compute();
	int startup_check();
	void initialize(int period);

private:
	std::shared_ptr < InnerModel > innerModel;
	bool startup_check_flag;
    bool camera_open = false;
    
    std::mutex my_mutex; 
    std::unique_ptr<RealSenseID::Preview> preview; 
    std::unique_ptr<PreviewRender> preview_callback; 

 
    vector<int> compression_params;
    vector<uchar> buffer;
    struct PARAMS
    {
        std::string device = "/dev/ttyACM0";
        bool display = false;
        bool compressed = false;
    };
    PARAMS pars;
} ;


////////////////////////////////Authentication/////////////////////////////////////
class MyAuthClbk : public RealSenseID::AuthenticationCallback
{
    std::vector<RealSenseID::FaceRect> _faces;
    size_t _result_idx = 0;
    unsigned int _ts =0;

public:
    // Face position
    struct UserData
    {
        string name;
        int x;
        int y;
        float angle;
    };
    vector<UserData> users_data;
	std::string userAuthenticated;
    vector<vector<cv::Point2i>> face_bounding_box;

    void OnResult(const RealSenseID::AuthenticateStatus status, const char* user_id) override
    {
        std::cout << "\n******* OnResult #" << _result_idx << "*******" << std::endl;
        users_data.resize(_result_idx+1);
        std::cout << "Status: " << status << std::endl;
        if (status == RealSenseID::AuthenticateStatus::Success)
        {
            std::cout << "******* Authenticate success.  user_id: " << user_id << " *******" << std::endl;
			std::string tmp_string(user_id);
			this->userAuthenticated=tmp_string;
            auto act_face_pose = _faces[_result_idx];
            UserData personData {.name=tmp_string, .x = act_face_pose.x + act_face_pose.w/2, .y = act_face_pose.y + act_face_pose.h/2};
            users_data[_result_idx] = personData;
//            act_bb.push_back(cv::Point2i(face.x, face.y));
//            act_bb.push_back(cv::Point2i(face.x + face.w, face.y));
//            act_bb.push_back(cv::Point2i(face.x, face.y + face.h));
//            act_bb.push_back(cv::Point2i(face.x + face.w, face.y + face.h));
//            face_bounding_box.push_back(act_bb);
            std::cout << "Face: " << "x: " << act_face_pose.x << " y: " << act_face_pose.y << " " << act_face_pose.w << "x" << act_face_pose.h << std::endl;
        }
        else
        {
            std::cout << "on_result: status: " << status << std::endl;
			this->userAuthenticated="Unknown";
        }
    }

    void OnHint(const RealSenseID::AuthenticateStatus hint) override
    {
        std::cout << "on_hint: hint: " << hint << std::endl;
    }
    void OnFaceDetected(const std::vector<RealSenseID::FaceRect>& faces, const unsigned int ts) override
    {
        _faces = faces;
        _ts = ts;
    }

    unsigned int GetLastTimeStamp()
    {
        return _ts;
    }

};


///////////////////////////////////Enroll////////////////////////////////
class MyEnrollClbk : public RealSenseID::EnrollmentCallback
{
    using FacePose = RealSenseID::FacePose;

public:
    void OnResult(const RealSenseID::EnrollStatus status) override
    {
        // std::cout << "on_result: status: " << status << std::endl;
        std::cout << "  *** Result " << status << std::endl;
    }

    void OnProgress(const FacePose pose) override
    {
        // find next pose that required(if any) and display instruction where to look
        std::cout << "  *** Detected Pose " << pose << std::endl;
        _poses_required.erase(pose);
        if (!_poses_required.empty())
        {
            auto next_pose = *_poses_required.begin();
            std::cout << "  *** Please Look To The " << next_pose << std::endl;
        }
    }

    void OnHint(const RealSenseID::EnrollStatus hint) override
    {
        std::cout << "  *** Hint " << hint << std::endl;
    }

private:
    std::set<RealSenseID::FacePose> _poses_required = {FacePose::Center, FacePose::Left, FacePose::Right};
};



#endif
