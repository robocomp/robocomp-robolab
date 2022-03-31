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
    auto left_x = std::stod(params.at("left_x").value);
    auto top_y = std::stod(params.at("top_y").value);
    auto width = std::stod(params.at("width").value);
    auto height = std::stod(params.at("height").value);
    auto tile = std::stod(params.at("tile").value);
    qInfo() << __FUNCTION__ << " Read parameters: " << left_x << top_y << width << height << tile;
    this->dimensions = QRectF(left_x, top_y, width, height);
    TILE_SIZE = tile;
    return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;

    viewer = new AbstractGraphicViewer(this->beta_frame, this->dimensions);
    this->resize(900,450);
    const auto &[rp, re] = viewer->add_robot(ROBOT_LENGTH, ROBOT_LENGTH);
    robot_polygon = rp;
    laser_in_robot_polygon = new QGraphicsRectItem(-10, 10, 20, 20, robot_polygon);
    laser_in_robot_polygon->setPos(0, 190);     // move this to abstract
    try
    {
        RoboCompGenericBase::TBaseState bState;
        differentialrobot_proxy->getBaseState(bState);
        last_point = QPointF(bState.x, bState.z);
    }
    catch(const Ice::Exception &e) { std::cout << e.what() << std::endl;}

    connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);
    connect(tilt_scrollbar, &QScrollBar::valueChanged, this, &SpecificWorker::new_tilt_value_slot);
    connect(sweep_button, &QPushButton::clicked, this, &SpecificWorker::sweep_button_slot);
    connect(trace_button, &QPushButton::clicked, this, &SpecificWorker::sweep_button_slot);

    // grid
    grid.initialize(dimensions, TILE_SIZE, &viewer->scene, false);

    this->Period = period;
	if(this->startup_check_flag)
		this->startup_check();
	else
		timer.start(Period);
}

void SpecificWorker::compute()
{
    //laser
    try
    {
        auto ldata = laser_proxy->getLaserData();
        draw_laser( ldata );
    }
    catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}

    //robot
//    try
//    {
//        RoboCompGenericBase::TBaseState bState;
//        differentialrobot_proxy->getBaseState(bState);
//        robot_polygon->setRotation(bState.alpha*180/M_PI);
//        robot_polygon->setPos(bState.x, bState.z);
//        if(sweep_button->isChecked())
//        {
//;           grid.setVisited(grid.pointToGrid(bState.x, bState.z), true);
//            sweep_lcdNumber->display(100.0 * grid.count_total_visited() / grid.count_total());
//        }
//        if(trace_button->isChecked())
//        {
//            QLineF line(last_point.x(), last_point.y(), bState.x, bState.z);
//            lines.push_back(viewer->scene.addLine(line, QPen(QColor("Blue"),40)));
//            last_point = QPointF(bState.x, bState.z);
//        }
//    }
//    catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}

    try
    {
        RoboCompFullPoseEstimation::FullPoseEuler bState;
        bState = fullposeestimation_proxy->getFullPoseEuler();
        qInfo()  << bState.x << bState.y << bState.rz;
        robot_polygon->setRotation(bState.rz*180/M_PI);
        robot_polygon->setPos(bState.x, bState.y);
        if(sweep_button->isChecked())
        {
            ;           grid.setVisited(grid.pointToGrid(bState.x, bState.y), true);
            sweep_lcdNumber->display(100.0 * grid.count_total_visited() / grid.count_total());
        }
        if(trace_button->isChecked())
        {
            QLineF line(last_point.x(), last_point.y(), bState.x, bState.y);
            lines.push_back(viewer->scene.addLine(line, QPen(QColor("Blue"),40)));
            last_point = QPointF(bState.x, bState.y);
        }
    }
    catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}

    // camera-tablet
    try
    {
        cv::Mat top_img_uncomp;
        QImage top_qimg;
        auto top_img = camerasimple_proxy->getImage();
        if(not top_img.image.empty())
        {
            if (top_img.compressed)
            {
                top_img_uncomp = cv::imdecode(top_img.image, -1);
                top_qimg = QImage(top_img_uncomp.data, top_img.width, top_img.height, QImage::Format_RGB888).scaled(
                        top_camera_label->width(), top_camera_label->height(), Qt::KeepAspectRatioByExpanding);;
            } else
                top_qimg = QImage(&top_img.image[0], top_img.width, top_img.height, QImage::Format_RGB888).scaled(
                        top_camera_label->width(), top_camera_label->height(), Qt::KeepAspectRatioByExpanding);;
            auto pix = QPixmap::fromImage(top_qimg);
            top_camera_label->setPixmap(pix);
        }
    }
    catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}
}

/////////////////////////////////////////////////////////////////////////
void SpecificWorker::draw_laser(const RoboCompLaser::TLaserData &ldata) // robot coordinates
{
    static QGraphicsItem *laser_polygon = nullptr;
    if (laser_polygon != nullptr)
        viewer->scene.removeItem(laser_polygon);

    QPolygonF poly;
    poly << QPointF(0,0);
    for(auto &&l : ldata)
        poly << QPointF(l.dist*sin(l.angle), l.dist*cos(l.angle));
    poly.pop_back();

    QColor color("LightGreen");
    color.setAlpha(40);
    laser_polygon = viewer->scene.addPolygon(laser_in_robot_polygon->mapToScene(poly), QPen(QColor("DarkGreen"), 30), QBrush(color));
    laser_polygon->setZValue(3);
}
void SpecificWorker::new_target_slot(QPointF target)
{
    qInfo() << __FUNCTION__ << " Received new target at " << target;
}
void SpecificWorker::new_tilt_value_slot(int value)
{
    try
    {
        float r_value = value * M_PI / 180;
        jointmotorsimple_proxy->setPosition( "tablet_joint", RoboCompJointMotorSimple::MotorGoalPosition{r_value, 1 });
    }
    catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}
}
void SpecificWorker::sweep_button_slot(bool checked)
{
    if(not checked)
    {
        grid.set_all_to_not_visited();
    }
}
void SpecificWorker::trace_button_slot(bool checked)
{
    if(not checked)
    {
        for(auto &l : lines)
            viewer->scene.removeItem(l);
        lines.clear();
    }
    else
    {
        try
        {
            RoboCompGenericBase::TBaseState bState;
            differentialrobot_proxy->getBaseState(bState);
            last_point = QPointF(bState.x, bState.z);
        }
        catch(const Ice::Exception &e) { std::cout << e.what() << std::endl;}
    }
}
////////////////////////////////////////////////////////////////////////////
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
// From the RoboCompCameraSimple you can call this methods:
// this->camerasimple_proxy->getImage(...)

/**************************************/
// From the RoboCompCameraSimple you can use this types:
// RoboCompCameraSimple::TImage

/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// this->differentialrobot_proxy->correctOdometer(...)
// this->differentialrobot_proxy->getBasePose(...)
// this->differentialrobot_proxy->getBaseState(...)
// this->differentialrobot_proxy->resetOdometer(...)
// this->differentialrobot_proxy->setOdometer(...)
// this->differentialrobot_proxy->setOdometerPose(...)
// this->differentialrobot_proxy->setSpeedBase(...)
// this->differentialrobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompJointMotorSimple you can call this methods:
// this->jointmotorsimple_proxy->getMotorParams(...)
// this->jointmotorsimple_proxy->getMotorState(...)
// this->jointmotorsimple_proxy->setPosition(...)
// this->jointmotorsimple_proxy->setVelocity(...)
// this->jointmotorsimple_proxy->setZeroPos(...)

/**************************************/
// From the RoboCompJointMotorSimple you can use this types:
// RoboCompJointMotorSimple::MotorState
// RoboCompJointMotorSimple::MotorParams
// RoboCompJointMotorSimple::MotorGoalPosition
// RoboCompJointMotorSimple::MotorGoalVelocity

/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser_proxy->getLaserAndBStateData(...)
// this->laser_proxy->getLaserConfData(...)
// this->laser_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData
