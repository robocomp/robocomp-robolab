/*
 *    Copyright (C) 2022 by YOUR NAME HERE
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
#include <qmath.h>
#include <cppitertools/enumerate.hpp>

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
    try
    {
        auto left_x = std::stod(params.at("left_x").value);
        auto top_y = std::stod(params.at("top_y").value);
        auto width = std::stod(params.at("width").value);
        auto height = std::stod(params.at("height").value);
        auto tile = std::stod(params.at("tile").value);
        qInfo() << __FUNCTION__ << " Read parameters: " << left_x << top_y << width << height << tile;
        viewer_dimensions = QRectF(left_x, top_y, width, height);
        consts.tile_size = tile;
    }
    catch(const std::exception &e){ std::cout << e.what() << std::endl;}

	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
        viewer = new AbstractGraphicViewer(this->beta_frame, this->viewer_dimensions);
        this->resize(900,650);
        const auto &[rp, re] = viewer->add_robot(consts.robot_length, consts.robot_length);
        robot_polygon = rp;
        laser_in_robot_polygon = new QGraphicsRectItem(-10, 10, 20, 20, robot_polygon);
        laser_in_robot_polygon->setPos(0, 190);     // move this to abstract

        local_grid.initialize(Local_Grid::Ranges{0, 360, 5}, Local_Grid::Ranges{0.f, 4000.f, 200}, &viewer->scene);

        // 3DViewer
        points = std::make_shared<std::vector<std::tuple<double, double, double>>>(512*256);
        colors = std::make_shared<std::vector<std::tuple<double, double, double>>>(512*256);
        viewer_3d = new Viewer(widget_3d, points, colors);
        viewer_3d->show();

        timer.start(100);
	}

}

void SpecificWorker::compute()
{
    RoboCompLaser::TLaserData ldata;
    try
	{
	  ldata = laser_proxy->getLaserData();
      std::vector<Eigen::Vector2f> ldata_polar(ldata.size());
      for(auto &&[i, p] : ldata | iter::enumerate)
          ldata_polar[i] = {p.angle, p.dist};
      local_grid.update_map_from_polar_data(ldata_polar, 4000);
    }
	catch(const Ice::Exception &e){std::cout << "Error reading from Camera" << e << std::endl;}

    cv::Mat rgb_frame;
    try
    {
        auto image = camerargbdsimple_proxy->getImage("/Giraff/neck/frame_base/Cuboid_frame_top/sphericalVisionRGBAndDepth/sensorRGB");
        if (image.width != 0 and image.height != 0)
        {
            rgb_frame = cv::Mat(cv::Size(image.width, image.height), CV_8UC3, &image.image[0], cv::Mat::AUTO_STEP);
            if (image.compressed)
                rgb_frame = cv::imdecode(image.image, -1);
        }
    }
    catch(const Ice::Exception &e){std::cout << "Error reading from cameraRGBDSimple" << std::endl;}

    struct RGB { uchar blue; uchar green; uchar red;  };
    try
    {
        // depth is coded in gray scale as 0-255  -> 0 - 500cm,  Eache gray level codes 2cm
        auto image = camerargbdsimple_proxy->getImage("/Giraff/neck/frame_base/Cuboid_frame_top/sphericalVisionRGBAndDepth/sensorDepth");
        if (image.width != 0 and image.height != 0)
        {
            cv::Mat frame(cv::Size(image.width, image.height), CV_8UC3, &image.image[0], cv::Mat::AUTO_STEP);
            if (image.compressed)
                frame = cv::imdecode(image.image, -1);
            cv::Mat gray_frame;
            cv::cvtColor(frame, gray_frame,  cv::COLOR_RGB2GRAY);
            // Let's assume that each column corresponds to a polar coordinate: ang_step = 360/image.width
            // and along the column we have radius
            // hor ang = 2PI/512 * i
            std::size_t i=0;
            int semi_height = image.height/2;
            float hor_ang, dist, x, y, z, proy;
            for(int u=0; u<image.height; u=u+3)
                for(int v=0; v<image.width; v++)
                {
                    hor_ang = 2*M_PI/image.width * v - M_PI; // cols to radians
                    dist = (float)gray_frame.ptr<uchar>(u)[v] * 19.f;  // pixel to dist scaling factor
                    if(dist > consts.max_camera_depth_range) continue;
                    dist /= 1000.f;  // to deca-meters?
                    x = -dist * sin(hor_ang);
                    y = dist * cos(hor_ang);
                    proy = dist * cos( atan2((semi_height - u), 128.f));
                    z = (semi_height - u)/128.f * proy; // 128 focal as PI fov angle for 256 pixels
                    z += consts.omni_camera_height_meters;
                    points->operator[](i) = std::make_tuple(x,y,z);
                    RGB& rgb = rgb_frame.ptr<RGB>(u)[v];
                    //colors->operator[](i++) = std::make_tuple(rgb.blue/255.0, rgb.green/255.0, rgb.red/255.0);
                    //float c = z/2;
                    colors->operator[](i++) = std::make_tuple(1.0,0.6,0.0);
                };
        }
        else qWarning() << __FUNCTION__ << "Empty image";
    }
    catch(const Ice::Exception &e){std::cout << "Error reading from cameraRGBDSimple" << std::endl;}

    // draw laser in 3D viewer
    for(size_t i = ldata.size(); const auto &p : ldata)
    {
        points->operator[](i) = std::make_tuple(p.dist*sin(p.angle)/1000.f, p.dist*cos(p.angle)/1000.f, 0.2);
        colors->operator[](i++) = std::make_tuple(0.0, 0.0, 1.0);
    }
    viewer->viewport()->repaint();
    viewer_3d->updateGL();
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

/**************************************/
// From the RoboCompCameraRGBDSimple you can call this methods:
// this->camerargbdsimple_proxy->getAll(...)
// this->camerargbdsimple_proxy->getDepth(...)
// this->camerargbdsimple_proxy->getImage(...)

/**************************************/
// From the RoboCompCameraRGBDSimple you can use this types:
// RoboCompCameraRGBDSimple::TImage
// RoboCompCameraRGBDSimple::TDepth
// RoboCompCameraRGBDSimple::TRGBD

/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser_proxy->getLaserAndBStateData(...)
// this->laser_proxy->getLaserConfData(...)
// this->laser_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

