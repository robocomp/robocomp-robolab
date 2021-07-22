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
#include "specificworker.h"

/**


* \brief Default constructor
*/

const char* PORT={"/dev/ttyACM0"};
//////////////////////////////Open port/////////////////////////////////////////
std::unique_ptr<RealSenseID::FaceAuthenticator> CreateAuthenticator(const char* port)
{
	RealSenseID::SerialConfig serial_config;
	serial_config.port = port;
    auto authenticator = std::make_unique<RealSenseID::FaceAuthenticator>();
    auto connect_status = authenticator->Connect(serial_config);
    if (connect_status != RealSenseID::Status::Ok)
    {
        std::cout << "Failed connecting to port " << serial_config.port << " status:" << connect_status << std::endl;
        std::exit(1);
    }
    std::cout << "Connected to device" << std::endl;
    return authenticator;
}

///////////////////////SPECIFICWORKER/////////////////////
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
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
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;

    RealSenseID::PreviewConfig config;
    this->preview = std::make_unique<RealSenseID::Preview>(config);
    this->preview_callback = std::make_unique<PreviewRender>();


	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		timer.start(Period);
	}

}

void SpecificWorker::compute()
{
	;
	
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

RoboCompCameraSimple::TImage SpecificWorker::CameraSimple_getImage()
{
    RoboCompCameraSimple::TImage frame;
    PreviewRender preview;
    preview=*preview_callback;
    frame.width =preview.frame.width;
    frame.height =preview.frame.height;
    frame.depth=3;
    for (int i =0;i<preview.frame.size;i++){
        frame.image.emplace_back(preview.frame.buffer[i]);
    }
    return frame;
}

RoboCompRealSenseFaceID::UserDataList SpecificWorker::RealSenseFaceID_authenticate()
{
//implementCODE
	RoboCompRealSenseFaceID::UserDataList dataList;
    RoboCompRealSenseFaceID::UserData data;
	auto authenticator = CreateAuthenticator(PORT);
    MyAuthClbk auth_clbk;
    auto status = authenticator->Authenticate(auth_clbk);
	if (status != RealSenseID::Status::Ok)
    {
        std::cout << "Status: " << status << std::endl << std::endl;
    }
	data.userAuthenticated=auth_clbk.userAuthenticated;
    dataList.emplace_back(data);
	return dataList;
}

bool SpecificWorker::RealSenseFaceID_enroll(std::string user)
{
//implementCODE

    if (user.size()>RealSenseID::FaceAuthenticator::MAX_USERID_LENGTH){
        return false;
    }
	auto authenticator = CreateAuthenticator(PORT);
    MyEnrollClbk enroll_clbk;
    
    auto status = authenticator->Enroll(enroll_clbk, user.c_str());
    if (status != RealSenseID::Status::Ok)
    {
        std::cout << "Status: " << status << std::endl << std::endl;
        return false;
    }
	status = authenticator->Standby();
    std::cout << "Status Save: " << status << std::endl << std::endl;
    if (status != RealSenseID::Status::Ok)
    {
        return false;
    }
    return true;
}

bool SpecificWorker::RealSenseFaceID_eraseAll()
{
	auto authenticator = CreateAuthenticator(PORT);
    auto auth_status = authenticator->RemoveAll();
    std::cout << "Final status:" << auth_status << std::endl << std::endl;
    if (auth_status != RealSenseID::Status::Ok)
    {
        return false;
    }
    return true;
}

bool SpecificWorker::RealSenseFaceID_eraseUser(std::string user)
{
	auto authenticator = CreateAuthenticator(PORT);
    auto auth_status = authenticator->RemoveUser(user.c_str());
    std::cout << "Final status:" << auth_status << std::endl << std::endl;
    if (auth_status != RealSenseID::Status::Ok)
    {
        return false;
    }
    return true;

}

RoboCompRealSenseFaceID::UserDataList SpecificWorker::RealSenseFaceID_getQueryUsers()
{
	RoboCompRealSenseFaceID::UserDataList dataList;
    RoboCompRealSenseFaceID::UserData data;
 	auto authenticator = CreateAuthenticator(PORT);

    unsigned int number_of_users = 0;
    auto status = authenticator->QueryNumberOfUsers(number_of_users);
    if (status != RealSenseID::Status::Ok)
    {
        std::cout << "Status: " << status << std::endl << std::endl;
        return dataList;
    }
    if (number_of_users == 0)
    {
        std::cout << "No users found" << std::endl << std::endl;
        return dataList;
    }

    // allocate needed array of user ids
    char** user_ids = new char*[number_of_users];
    for (unsigned i = 0; i < number_of_users; i++)
    {
        user_ids[i] = new char[RealSenseID::FaceAuthenticator::MAX_USERID_LENGTH];
    }
    unsigned int nusers_in_out = number_of_users;
    status = authenticator->QueryUserIds(user_ids, nusers_in_out);
    if (status != RealSenseID::Status::Ok)
    {
        std::cout << "Status: " << status << std::endl << std::endl;
        // free allocated memory and return on error
        for (unsigned int i = 0; i < number_of_users; i++)
        {
            delete user_ids[i];
        }
        delete[] user_ids;
        return dataList;
    }

    std::cout << std::endl << nusers_in_out << " Users:\n==========\n";
    for (unsigned int i = 0; i < nusers_in_out; i++)
    {
        std::cout << (i + 1) << ".  " << user_ids[i] << std::endl;
        std::string tmp_string(user_ids[i]);
		data.userAuthenticated=tmp_string;
        dataList.emplace_back(data);
    }

    std::cout << std::endl;

    // free allocated memory
    for (unsigned int i = 0; i < number_of_users; i++)
    {
        delete user_ids[i];
    }
    delete[] user_ids;

	return dataList;
}

bool SpecificWorker::RealSenseFaceID_startPreview()
{
//implementCODEs
    return this->preview->StartPreview(*preview_callback);

}




bool SpecificWorker::RealSenseFaceID_stopPreview()
{
//implementCODE
    return this->preview->StopPreview();

}

/**************************************/
// From the RoboCompCameraSimple you can use this types:
// RoboCompCameraSimple::TImage

/**************************************/
// From the RoboCompRealSenseFaceID you can use this types:
// RoboCompRealSenseFaceID::UserData
