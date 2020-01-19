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
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

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
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		std::string innermodel_path = par.value;
		innerModel = std::shared_ptr<InnerModel>(new InnerModel(innermodel_path));
	}
	catch(const std::exception &e)
    {
        qFatal("Error reading config params");
    }

	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
    fx = 545;  // entrance
    fy = 545;
    
    fx = 462;  // VREP CAMERAS
    fy = 462;
    
    name = "0";
    this->Period = 200;
    timer.setSingleShot(true);
	timer.start(Period);
}


void SpecificWorker::compute()
{    
    qDebug() << "hola";
    RoboCompAprilTagsServer::tagsList tagsList;
    RoboCompCameraRGBDSimple::TImage simple;
    RoboCompRGBD::imgType rgbd;
    RoboCompRGBD::depthType depth;
    RoboCompJointMotor::MotorStateMap motor;

    //if(input = "SimpleCamera")
    //   april_frame = getSimpleCamera();
    //if(input = "RGBD")
    //    april_frame = getRGBD();

    auto s = simple.width;
    try
    {
        camerargbdsimple_proxy->getImage(simple);
    }
    catch(const Ice::Exception& e)
    {
        std::cerr << e.what() << '\n';
        return;
    }

    cv::Mat frame_m(simple.width, simple.height, CV_8UC3);
    cv::Mat frame(simple.width, simple.height, CV_8UC3);  

    //qDebug() << " Read image: " << simple.width << simple.height << simple.depth;
    memcpy(&frame_m.data[0], &(simple.image[0]), simple.width*simple.height*simple.depth);
    cv::cvtColor(frame_m, frame_m, CV_BGR2RGB);
    cv::flip(frame_m,frame_m, 0);  // estaba a 1 !!!!!!!!!
    
    RoboCompAprilTagsServer::Image april_frame;
    april_frame.frmt.modeImage=RoboCompAprilTagsServer::RGB888Packet;
    april_frame.frmt.width=simple.width;
    april_frame.frmt.height=simple.height;
    april_frame.frmt.size=simple.depth;
    april_frame.data.resize(april_frame.frmt.width*april_frame.frmt.height*april_frame.frmt.size);
    april_frame.data = simple.image;
    //memcpy(&april_frame.data[0], &frame_m.data[0], april_frame.frmt.width*april_frame.frmt.height*april_frame.frmt.size);
    //tagsList = apriltagsserver_proxy->getAprilTags(frame, 384, fx, fy);
    try
    {
        //tagsList = apriltagsserver_proxy->getAprilTags(april_frame, 350, fx, fy);
        // 280 is the part that occupies the black squeare
        tagsList = apriltagsserver_proxy->getAprilTags(april_frame, 280, fx, fy);
    }
    catch(const Ice::Exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
    cv::line(frame_m,cv::Point(320,0),cv::Point(320,480),cv::Scalar(255,0,0),1);	
    cv::line(frame_m,cv::Point(0,240),cv::Point(640,240),cv::Scalar(255,0,0),1);
    // for (int ir=0; ir<480; ir++)
    // {
    //     for (int ic=0; ic<640; ic++)
    //     {
    //         frame_m.at<uint8_t>(ir, 640-ic, 2) = int(255.*depth[ic + 640*ir] / 10000.);
    //     }
    // }
    //cv::imshow("Calib", frame_m);
    if (tagsList.size() > 0)
    {
        qDebug() << "TAG:";
        qDebug() << "   " << tagsList[0].tx << tagsList[0].ty << tagsList[0].tz;
        qDebug() << "   " << tagsList[0].rx << tagsList[0].ry << tagsList[0].rz;
        qDebug() << "------------------------------------------";

        innerModel->updateTransformValuesS(std::string("april_raw_")+name, tagsList[0].tx, tagsList[0].ty, tagsList[0].tz, tagsList[0].rx, tagsList[0].ry, tagsList[0].rz);
        QVec tr = innerModel->transformS6D(std::string("april_")+name, std::string("cam_")+name);
        innerModel->updateTransformValuesS(std::string("cam_")+name, tr(0), tr(1), tr(2), tr(3), tr(4), tr(5));
        QVec tt = innerModel->transformS6D(std::string("april_")+name, std::string("cam_")+name);
        QVec ts = innerModel->transformS6D(std::string("world"), std::string("cam_")+name);
        qDebug() << "tt: " <<  "tx=" << tt(0) << "ty=" << tt(1) << "tz=" << tt(2) << "rx=" << tt(3) << "ry=" << tt(4) << "rz=" << tt(5);
        qDebug() << "Camera coordinates in World: " <<  "tx=" << ts(0) << "ty=" << ts(1) << "tz=" << ts(2) << "rx=" << ts(3) << "ry=" << ts(4) << "rz=" << ts(5);
        float d1 = QVec::vec3(tr(0), tr(1), tr(2)).norm2();
        qDebug() << "Dist from camera to World origin: " << d1;
    }
}


// void SpecificWorker::compute()
// {
//     static cv::Mat frame_m(480, 640, CV_8UC3);
//     static DepthSeq depth;
//     static imgType color;
//     RoboCompJointMotor::MotorStateMap hState;
//     RoboCompGenericBase::TBaseState bState;
//     RoboCompAprilTagsServer::tagsList tagsList;

//     for (int i=0; i<NUMBER_OF_CAMERAS; i++)
//     {
//         //camera_data[i].rgbd_proxy->getData(color, depth, hState, bState);
//         RoboCompCameraRGBDSimple::TImage simple;
//         camerargbdsimple_proxy->getImage(simple);
        
//         //memcpy(&frame_m.data[0], &(color[0]), 640*480*3);
//         memcpy(&frame_m.data[0], &(simple.image[0]), 640*480*3);
        
//         cv::cvtColor(frame_m, frame_m, CV_BGR2RGB);
//         cv::flip(frame_m,frame_m, 1);
//         memcpy(&frame.data[0], &frame_m.data[0], 640*480*3);
//         tagsList = apriltagsserver_proxy->getAprilTags(frame, 384, camera_data[i].mfx, camera_data[i].mfy);
//         cv::line(frame_m,cv::Point(320,0),cv::Point(320,480),cv::Scalar(255,0,0),1);	
//         cv::line(frame_m,cv::Point(0,240),cv::Point(640,240),cv::Scalar(255,0,0),1);
//         // for (int ir=0; ir<480; ir++)
//         // {
//         //     for (int ic=0; ic<640; ic++)
//         //     {
//         //         frame_m.at<uint8_t>(ir, 640-ic, 2) = int(255.*depth[ic + 640*ir] / 10000.);
//         //     }
//         // }
//         cv::imshow(camera_data[i].name, frame_m);
//         if (tagsList.size() > 0)
//         {
// //             printf("0: t=(%f, %f, %f)  r=(%f, %f, %f)\n", tagsList[0].tx, tagsList[0].ty, tagsList[0].tz, tagsList[0].rx, tagsList[0].ry, tagsList[0].rz);
//             innerModel->updateTransformValuesS(std::string("april_raw_")+camera_data[i].name, tagsList[0].tx, tagsList[0].ty, tagsList[0].tz, tagsList[0].rx, tagsList[0].ry, tagsList[0].rz);
//             QVec tr = innerModel->transformS6D(std::string("april_")+camera_data[i].name, std::string("cam_")+camera_data[i].name);
//             innerModel->updateTransformValuesS(std::string("cam_")+camera_data[i].name, tr(0), tr(1), tr(2), tr(3), tr(4), tr(5));
//             QVec tt = innerModel->transformS6D(std::string("april_")+camera_data[i].name, std::string("cam_")+camera_data[i].name);
//             printf("CAM %s from the TAG: (%f %f %f)  (%f %f %f)\n", camera_data[i].name.c_str() , tr(0), tr(1), tr(2), tr(3), tr(4), tr(5));
//             float d1 = QVec::vec3(tr(0), tr(1), tr(2)).norm2();
//             printf("%f\n", d1);
            
// // window cam: ground truth 3922mm
// // 460 - 3300
// // 500 - 3571
// // 518 - 3693
// // 531 - 3780
// // 545 - 3888
// // 551 - 3922 **
// // 552 - 3932
// // 555 - 3953
// // 559 - 3980
// // 565 - 4022
// // fridge cam: 3116mm
// // entrance cam: 3752mm
// // 545 - 3755
// // 548 - 3775
// // 550 - 3789
// // 551 - 3798            
// //             
            
// //             float d2 = depth[tagsList[0].cx+ 640*tagsList[0].cy];
// //             if (d2 > 1)
// //             {
// //                 printf("  Ratio:   %f   %f\n", d1, d2);
// //                 camera_data[i].mfx *= 0.99 + 0.01*d2/d1;
// //                 camera_data[i].mfy *= 0.99 + 0.01*d2/d1;
// //                 printf("  Focal %s  (%f, %f)\n", camera_data[i].name.c_str(), camera_data[i].mfx, camera_data[i].mfy);
// //             }
//         }
//     }
// }


