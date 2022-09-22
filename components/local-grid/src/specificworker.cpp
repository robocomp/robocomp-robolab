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
#include <cppitertools/enumerate.hpp>
#include <cppitertools/range.hpp>
#include <cppitertools/zip.hpp>

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
        points = std::make_shared<std::vector<std::tuple<float, float, float>>>();
        colors = std::make_shared<std::vector<std::tuple<float, float, float>>>();
        colors_cog = std::make_shared<std::vector<std::tuple<float, float, float>>>();
        planes = std::make_shared<std::vector<std::tuple<point3f, point3f, point3f, point3f>>>();
        viewer_3d = new Viewer(widget_3d, points, colors, planes);
        viewer_3d->show();
        viewer_3d_cog = new Viewer_Cog(widget_3d_cog, planes, colors_cog);
        viewer_3d_cog->show();

        // central camera transform
        Rt = Eigen::Translation<float, 3>(0.0, 0.1, 1.68);
        Rt.rotate(Eigen::AngleAxis<float>(qDegreesToRadians(-22.0), Eigen::Vector3f::UnitX()));

        // get object names and joints_list
        try
        {
            yolo_object_names = yoloobjects_proxy->getYoloObjectNames();
            yolo_joint_data = yoloobjects_proxy->getYoloJointData();
        }
        catch(const Ice::Exception &e) {std::cout << e.what() << " Error connecting with YoloObjects interface to retrieve names" << std::endl;}

        timer.start(33);

        // Concepts
        //Concept cp = {.active=false, .children={"wall_1","wall_2","wall_3","wall_4"}};
        //room_grammar.emplace("floor", cp);
//        try
//        {
//            std::vector<std::string> class_names = yoloserver2_proxy->getClassNames();
//            std::vector<cv::Scalar> class_colors;
//            for(const auto n: class_names)
//                class_colors.emplace_back(cv::Scalar(1,2,3));
//        }
//        catch (const Ice::Exception &e){ std::cout << e.what() << "Aborting" << std::endl; std::terminate();}

        connect(&timer_cog, SIGNAL(timeout()), this, SLOT(compute_cog()));
        std::cout << "End initializing worker" << std::endl;
        //timer_cog.start();
    }

}
void SpecificWorker::compute()
{
    points->clear(); colors->clear(); colors_cog->clear(); planes->clear();

    // read raw sensors
    //const auto &[ldata, ldata_polar] = read_laser();
    const auto &[omni_rgb_frame, _] = read_rgb("/Shadow/omnicamera/sensorRGB");
    auto omni_depth_frame = read_depth_omni();
    //const auto &[central_rgb, central_rgb_focal] = read_rgb("camera_top");
    //const auto &[central_depth, central_depth_focal] = read_depth("camera_top");

    // get 3D points from sensors
    get_omni_3d_points(omni_depth_frame, omni_rgb_frame);
    //get_laser_3d_points(ldata, std::make_tuple(0.0, 1.0, 0.0));
    //get_central_3d_points(central_depth, central_rgb, central_rgb_focal);

    // Group 3D points by angular sectors and sorted by minimum distance
    sets.clear();
    sets = group_by_angular_sectors(false);

    // update local_grid
    //local_grid.update_map_from_polar_data(ldata_polar, 4000);
    //local_grid.update_map_from_3D_points(points);

    // compute floor line
    auto floor_line = compute_floor_line(sets);

    // estimate floor rectangle from floor_line
    //estimate_floor_object(floor_line);

    // get Yolo coordinates
//    try
//    {
//        auto yolo_data = yoloobjects_proxy->getYoloObjects();
//        draw_objects(yolo_data.objects, central_rgb);
//        draw_people(yolo_data.people, central_rgb);
//
//        cv::imshow("central camera", central_rgb);
//        cv::waitKey(1);
//    }
//    catch(const Ice::Exception &e){ std::cerr << e.what() << ". No response from YoloServer" << std::endl;};

    // update viewers
    viewer->viewport()->repaint();
    viewer_3d->updateGL();
    viewer_3d_cog->updateGL();

    fps.print("FPS:");
}

////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::draw_objects(const RoboCompYoloObjects::TObjects &objects, cv::Mat img)
{
    // Plots one bounding box on image img
    //qInfo() << __FUNCTION__ << QString::fromStdString(box.name) << box.left << box.top << box.right << box.bot << box.prob;
    for(const auto &box: objects)
    {
        auto tl = round(0.002 * (img.cols + img.rows) / 2) + 1; // line / fontthickness
        cv::Scalar color(0, 255, 0); // box color
        cv::Point c1(box.left, box.top);
        cv::Point c2(box.right, box.bot);
        cv::rectangle(img, c1, c2, color, tl, cv::LINE_AA);
        int tf = (int) std::max(tl - 1, 1.0);  // font thickness
        int baseline = 0;
        std::string label = yolo_object_names.at(box.type) + " " + std::to_string((int) (box.prob * 100)) + "%";
        auto t_size = cv::getTextSize(label, 0, tl / 3.f, tf, &baseline);
        c2 = {c1.x + t_size.width, c1.y - t_size.height - 3};
        cv::rectangle(img, c1, c2, color, -1, cv::LINE_AA);  // filled
        cv::putText(img, label, cv::Size(c1.x, c1.y - 2), 0, tl / 3, cv::Scalar(225, 255, 255), tf, cv::LINE_AA);
    }
}
void SpecificWorker::draw_people(const RoboCompYoloObjects::TPeople &people, cv::Mat img)
{
    qInfo() << __FUNCTION__ << people.size();
    for(const auto &person: people)
    {
        int num_landmarks = person.joints.size();
        // Draws the connections if the start and end landmarks are both visible.
        for(const auto &[first, second] : yolo_joint_data.connections)
        {
            if (not(first <=0 and first > num_landmarks and second <= 0 and second > num_landmarks))
                qWarning() << "Landmark index is out of range. Invalid connection from landmark" << first << " to landmark ";
            if (person.joints.contains(first) and person.joints.contains(second))
                cv::line(img, cv::Point(person.joints.at(first).i, person.joints.at(first).j),
                         cv::Point(person.joints.at(second).i, person.joints.at(second).j), cv::Scalar(0, 128, 0), 4);
        }
        // draw circles on joints
        for(const auto &[id, jnt]: person.joints)
            cv::circle(img, cv::Point(jnt.i, jnt.j), 3, cv::Scalar(255, 255, 255), 2);
    }
}
std::tuple<RoboCompLaser::TLaserData, std::vector<Eigen::Vector2f>> SpecificWorker::read_laser()
{
    std::vector<Eigen::Vector2f> ldata_polar;
    RoboCompLaser::TLaserData ldata;
    try
    {
        ldata = laser_proxy->getLaserData();
        ldata_polar.resize(ldata.size());
        for(auto &&[i, p] : ldata | iter::enumerate)
            ldata_polar[i] = {p.angle, p.dist};
    }
    catch(const Ice::Exception &e){std::cout << "Error reading from Camera" << e << std::endl;}
    return std::make_tuple(ldata, ldata_polar);
}
std::tuple<cv::Mat, float> SpecificWorker::read_rgb(const std::string &camera_name)
{
    cv::Mat rgb_frame;
    RoboCompCameraRGBDSimple::TImage image;
    try
    {
        image = camerargbdsimple_proxy->getImage(camera_name);
        if (image.width != 0 and image.height != 0)
        {
            rgb_frame = cv::Mat(cv::Size(image.width, image.height), CV_8UC3, &image.image[0], cv::Mat::AUTO_STEP);
            if (image.compressed)
                rgb_frame = cv::imdecode(image.image, -1);
        }
        else qWarning() << __FUNCTION__ << "Empty image";

    }
    catch(const Ice::Exception &e){std::cout << "Error reading from cameraRGBDSimple at " << camera_name << std::endl;}
    return std::make_tuple(rgb_frame.clone(), image.focaly);

}
std::tuple<cv::Mat, float> SpecificWorker::read_depth(const std::string &camera_name)
{
    cv::Mat depth_frame;
    RoboCompCameraRGBDSimple::TDepth image;
    try
    {
        image = camerargbdsimple_proxy->getDepth(camera_name);
        if (image.width != 0 and image.height != 0)
        {
            depth_frame = cv::Mat(cv::Size(image.width, image.height), CV_32FC1, &image.depth[0], cv::Mat::AUTO_STEP);
            if (image.compressed)
                depth_frame = cv::imdecode(image.depth, -1);
            //depth_frame.convertTo(depth_frame, CV_8UC3, 255. / 10, 0);
            //applyColorMap(depth_frame, depth_frame, cv::COLORMAP_RAINBOW); //COLORMAP_HSV tb
            //cv::imshow("Depth image ", depth_frame);
        }
        else qWarning() << __FUNCTION__ << "Empty image";
    }
    catch(const Ice::Exception &e){std::cout << "Error reading from cameraRGBDSimple (DEPTH)" << std::endl;}
    return std::make_tuple(depth_frame.clone(), image.focalx);
}
cv::Mat SpecificWorker::read_depth_omni()
{
    cv::Mat gray_frame;
    try
    {
        // depth is coded in gray scale as 0-255  -> 0 - 500cm,  Eache gray level codes 2cm
        auto image = camerargbdsimple_proxy->getImage("/Shadow/omnicamera/sensorDepth");
        if (image.width != 0 and image.height != 0)
        {
            cv::Mat frame(cv::Size(image.width, image.height), CV_8UC3, &image.image[0], cv::Mat::AUTO_STEP);
            if (image.compressed)
                frame = cv::imdecode(image.image, -1);
            cv::cvtColor(frame, gray_frame, cv::COLOR_RGB2GRAY);
            //qInfo() << gray_frame.ptr<uchar>(10)[20] << gray_frame.ptr<uchar>(100)[100] << gray_frame.ptr<uchar>(200)[200];
        }
        else qWarning() << __FUNCTION__ << "Empty image";
    }
    catch (const Ice::Exception &e)
    { std::cout << "Error reading from cameraRGBDSimple at " << std::endl; }
    return gray_frame.clone();
}

////////////////////////// 3D Points /////////////////////////////////////////////////////////
void SpecificWorker::get_omni_3d_points(const cv::Mat &depth_frame, const cv::Mat &rgb_frame)
{
    // Let's assume that each column corresponds to a polar coordinate: ang_step = 360/image.width
    // and along the column we have radius
    // hor ang = 2PI/512 * i
    std::size_t i = points->size();
    points->resize(depth_frame.rows * depth_frame.cols);
    colors->resize(points->size());
    int semi_height = depth_frame.rows/2;
    float hor_ang, dist, x, y, z, proy, ang_slope = 2*M_PI/depth_frame.cols;
    int cont = 0;
    for(int u=50; u<depth_frame.rows-50; u=u+1)
        for(int v=0; v<depth_frame.cols; v++)
        {
            hor_ang = ang_slope * v - M_PI; // cols to radians
            dist = (float)depth_frame.ptr<uchar>(u)[v] * 19.f;  // pixel to dist scaling factor -> to mm
            //if(std::isnan(dist)) cont++;
            if(dist > consts.max_camera_depth_range) continue;
            if(dist < consts.min_camera_depth_range) continue;
            dist /= 1000.f; // to meters
            x = -dist * sin(hor_ang);
            y = dist * cos(hor_ang);
            proy = dist * cos( atan2((semi_height - u), 128.f));
            z = (semi_height - u)/128.f * proy; // 128 focal as PI fov angle for 256 pixels
            z += consts.omni_camera_height_meters;
            points->operator[](i) = std::make_tuple(x,y,z);
            auto rgb = rgb_frame.ptr<cv::Vec3b>(u)[v];
            colors->operator[](i++) = std::make_tuple(rgb[0]/255.0, rgb[1]/255.0, rgb[2]/255.0);
            colors->operator[](i++) = std::make_tuple(1.0,0.0,0.6);
        };
}
void SpecificWorker::get_laser_3d_points(const RoboCompLaser::TLaserData &ldata, const std::tuple<float, float, float> &color)
{
    size_t i = points->size();
    points->resize(points->size() + ldata.size());
    colors->resize(points->size());
    for(const auto &p : ldata)
    {
        points->operator[](i) = std::make_tuple(p.dist*sin(p.angle)/1000.f, p.dist*cos(p.angle)/1000.f + 0.172, 0.2); // laser offset on robot
        colors->operator[](i++) = color;
    }
}
void SpecificWorker::get_central_3d_points(const cv::Mat &central_depth, const cv::Mat &central_rgb, float focal)
{
    // compute X,Y,Z coordinates from RGBD images
    size_t i = points->size();
    size_t step = 2;
    points->resize(points->size() + central_depth.rows/step * central_depth.cols/step);
    colors->resize(points->size());

    for(int u=0; u<central_depth.rows; u=u+step)
        for(int v=0; v<central_depth.cols; v=v+step)
        {
            int rows = (central_depth.rows / 2) - u;
            int cols = (central_depth.cols / 2) - v;
            // compute axis coordinates according to the camera's coordinate system (Y outwards and Z up)
            float y = central_depth.ptr<float>(u)[v];
            float x = -cols * y / focal;  // x axis is inverted
            float z = rows * y / focal;
            auto res = Rt * Eigen::Vector3f(x, y, z);
            auto rgb = central_rgb.ptr<cv::Vec3b>(u)[v];
            points->operator[](i) = std::make_tuple(res.x(), res.y(), res.z());
            colors->operator[](i++) = std::make_tuple(rgb[0]/255.f, rgb[1]/255.f, rgb[2]/255.f);
            //colors->operator[](i++) = std::make_tuple(1.0, 0.6, 0.0);
        }
}

/////////////////////////  Group  /////////////////////////////////////////////////
SpecificWorker::SetsType SpecificWorker::group_by_angular_sectors(bool draw)
{
    int num_ang_bins = 360;
    const double ang_bin = 2.0*M_PI/num_ang_bins;
    SetsType sets(num_ang_bins);
    for(const auto &[point, color] : iter::zip(*points, *colors))
    {
        const auto &[x,y,z] = point;
        int ang_index = floor((M_PI + atan2(x, y))/ang_bin);
        float dist = sqrt(x*x+y*y+z*z);
        if(dist < 0.1 or dist > 4) continue;
        if(z < 0.4 or fabs(z)>2) continue;
        sets[ang_index].emplace(std::make_tuple(Eigen::Vector3f(x,y,z), color));
    }

    // regenerate points and colors
    points->clear(); colors->clear();
    if(draw)
    {
        for (const auto &s: sets)
            if (not s.empty())
                for (auto init = s.cbegin(); init != s.cend(); ++init)
                {
                    const auto &[point, color] = *init;
                    for (auto &&t: iter::range(0.f, point.z(), 0.02f))
                    {
                        points->emplace_back(std::make_tuple(point.x(), point.y(), t));
                        colors->emplace_back(color);
                    }
                }
    }
    return sets;
}
vector<Eigen::Vector2f> SpecificWorker::compute_floor_line(SpecificWorker::SetsType &sets, bool draw)
{
    points->clear(); colors->clear();
    vector<Eigen::Vector2f> floor_line;
    for(const auto &s: sets)
    {
        if(s.empty()) continue;
        Eigen::Vector3f p = get<Eigen::Vector3f>(*s.cbegin());
        floor_line.emplace_back(Eigen::Vector2f(p.x(), p.y()));
        if(draw)
        {
            points->emplace_back(make_tuple(p.x(), p.y(), p.z()));
            colors->push_back({1.0, 1.0, 0.0});
        }
        //qInfo() << __FUNCTION__ << "[" << p.x()*1000 << p.y()*1000 << "]";
    }
    //qInfo() << __FUNCTION__ << "--------------------------------------";
    return floor_line;
}
cv::RotatedRect SpecificWorker::estimate_floor_object(const vector<Eigen::Vector2f> &floor_line) const
{
    if(floor_line.empty()) return cv::RotatedRect();

    // computer min area rect including all points
    std::vector<cv::Point2f> cv_points(floor_line.size());
    for(size_t i=0; const auto &p : floor_line)
        cv_points[i++] = cv::Point2f{p.x(), p.y()};
    cv::RotatedRect floor = cv::minAreaRect(cv_points);

    // emplace floor plane
    planes->clear();
    cv::Point2f pts[4];
    floor.points(pts);
    planes->emplace_back(std::make_tuple(std::make_tuple(pts[0].x, pts[0].y, 0.f),
                                         std::make_tuple(pts[1].x, pts[1].y, 0.f),
                                         std::make_tuple(pts[2].x, pts[2].y, 0.f),
                                         std::make_tuple(pts[3].x, pts[3].y, 0.f)));
    //colors_cog->emplace_back(std::make_tuple(rgb[0]/255.0, rgb[1]/255.0, rgb[2]/255.0));

    // create walls
    planes->emplace_back(std::make_tuple(std::make_tuple(pts[0].x, pts[0].y, 0.f),
                                         std::make_tuple(pts[0].x, pts[0].y, 1.5f),
                                         std::make_tuple(pts[1].x, pts[1].y, 1.5f),
                                         std::make_tuple(pts[1].x, pts[1].y, 0.f)));

    planes->emplace_back(std::make_tuple(std::make_tuple(pts[1].x, pts[1].y, 0.f),
                                         std::make_tuple(pts[1].x, pts[1].y, 1.5f),
                                         std::make_tuple(pts[2].x, pts[2].y, 1.5f),
                                         std::make_tuple(pts[2].x, pts[2].y, 0.f)));

    planes->emplace_back(std::make_tuple(std::make_tuple(pts[2].x, pts[2].y, 0.f),
                                         std::make_tuple(pts[2].x, pts[2].y, 1.5f),
                                         std::make_tuple(pts[3].x, pts[3].y, 1.5f),
                                         std::make_tuple(pts[3].x, pts[3].y, 0.f)));

    planes->emplace_back(std::make_tuple(std::make_tuple(pts[3].x, pts[3].y, 0.f),
                                         std::make_tuple(pts[3].x, pts[3].y, 1.5f),
                                         std::make_tuple(pts[0].x, pts[0].y, 1.5f),
                                         std::make_tuple(pts[0].x, pts[0].y, 0.f)));

    return floor;
}

////////////////////////////////////////////////////////////////////////////////
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

