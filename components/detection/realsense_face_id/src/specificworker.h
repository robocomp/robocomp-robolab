/*
 *    Copyright (C) 2021 by YOUR NAME HERE
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
	dsf
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

#include "RealSenseID/Preview.h"
#include "RealSenseID/FaceAuthenticator.h"
#include <iostream>
#include <stdio.h>

///////////////////////////////IMAGE////////////////////////////
class PreviewRender : public RealSenseID::PreviewImageReadyCallback
{
public:
		RealSenseID::Image frame;
		void OnPreviewImageReady(const RealSenseID::Image image){
		std::cout << "frame #" << image.number << ": " << image.width << "x" << image.height << " (" << image.size
					<< " bytes)" << "stride: "<< image.stride << std::endl;
		this->frame=image;
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



public slots:
	void compute();
	int startup_check();
	void initialize(int period);

private:
	std::shared_ptr < InnerModel > innerModel;
	bool startup_check_flag;

    std::unique_ptr<RealSenseID::Preview> preview;
	std::unique_ptr<PreviewRender> preview_callback;
} ;


////////////////////////////////Authentication/////////////////////////////////////
class MyAuthClbk : public RealSenseID::AuthenticationCallback
{
public:
	std::string userAuthenticated;
    void OnResult(const RealSenseID::AuthenticateStatus status, const char* user_id) override
    {
        if (status == RealSenseID::AuthenticateStatus::Success)
        {
            std::cout << "******* Authenticate success.  user_id: " << user_id << " *******" << std::endl;
			std::string tmp_string(user_id);
			this->userAuthenticated=tmp_string;
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
