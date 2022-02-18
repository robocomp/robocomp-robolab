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
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }






	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	this->startCamera=false;
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
	std::string input;
	RoboCompRealSenseFaceID::UserDataList dataList;
	RoboCompRealSenseFaceID::UserData data;
	RoboCompCameraSimple::TImage frame;
	std::cout<<endl<<"a autentificar"<<endl<< "d borrar paco"<<endl<< "b borrar todo"
	<<endl<< "y arrancar camara"
	<<endl<< "n para camara"<<
	endl<< "visualizar camara"
	<<endl<< "u lista de usuarios"<<endl;
	
	
	
	
	if (std::getline(std::cin, input)){
        if (input.empty() || input.length() > 1)
            ;
		else{
			char key = input[0];

			switch (key)
			{
			case 'e':{
				std::string user_id;
				do
				{
					std::cout << "User id to enroll: ";
					std::getline(std::cin, user_id);
            	} while (user_id.empty());
				std::cout<<this->realsensefaceid_proxy->enroll(user_id)<<std::endl;
				break;}
			case 'a':{
				dataList=this->realsensefaceid_proxy->authenticate();
				for (int i=0;i<dataList.size();i++){
					data=dataList.at(i);
					std::cout<<"["<<i<<"] "<<data.userAuthenticated<<std::endl;
				}
				break;}
			case 'd':
				std::cout<<this->realsensefaceid_proxy->eraseUser("Paco")<<std::endl;
				break;	
			case 'b':
				std::cout<<this->realsensefaceid_proxy->eraseAll()<<std::endl;
				break;
			case 'y':{
				std::cout<<this->realsensefaceid_proxy->startPreview()<<std::endl;
				std::cout<<"arrancamos"<<std::endl;
				this->startCamera=true;
				break;}
			case 'n':{
				std::cout<<this->realsensefaceid_proxy->stopPreview()<<std::endl;
				this->startCamera=false;
				break;}
			case 'c':{
				if(this->startCamera){
					frame = this->camerasimple_proxy->getImage();
					std::cout << "depth #" << frame.depth << ": " << frame.width << "x" << frame.height << std::endl;
					for (int i = 0; i < 60; i=i+3)
					{
						printf("%d ",frame.image.at(i));
					}
					
				}
				break;}
			case 'u':{
				dataList=this->realsensefaceid_proxy->getQueryUsers();
				for (int i=0;i<dataList.size();i++){
					data=dataList.at(i);
					std::cout<<"["<<i<<"] "<<data.userAuthenticated<<std::endl;
				}
				break;}
		}}}
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}




/**************************************/
// From the RoboCompCameraSimple you can call this methods:
// this->camerasimple_proxy->getImage(...)

/**************************************/
// From the RoboCompCameraSimple you can use this types:
// RoboCompCameraSimple::TImage

/**************************************/
// From the RoboCompRealSenseFaceID you can call this methods:
// this->realsensefaceid_proxy->authenticate(...)
// this->realsensefaceid_proxy->enroll(...)
// this->realsensefaceid_proxy->eraseAll(...)
// this->realsensefaceid_proxy->eraseUser(...)
// this->realsensefaceid_proxy->getQueryUsers(...)
// this->realsensefaceid_proxy->startPreview(...)
// this->realsensefaceid_proxy->stopPreview(...)

/**************************************/
// From the RoboCompRealSenseFaceID you can use this types:
// RoboCompRealSenseFaceID::UserData


