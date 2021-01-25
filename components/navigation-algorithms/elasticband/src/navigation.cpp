//
// Created by robolab on 17/01/20.
//

#include "navigation.h"
#include <QGraphicsLineItem>
#include <ranges>

template<typename TMap, typename TController>
void Navigation<TMap,TController>::initialize( const Grid<>::Dimensions &dim, std::shared_ptr<InnerModel> innerModel_,
                             std::shared_ptr<RoboCompCommonBehavior::ParameterList> configparams_,
                             RoboCompOmniRobot::OmniRobotPrxPtr omnirobot_proxy_,
                             QGraphicsScene *scene_)
{
    //qDebug() << "Navigation - " << __FUNCTION__;
    innerModel = innerModel_;
    configparams = configparams_;
    scene = scene_;
    omnirobot_proxy = omnirobot_proxy_;
    stopRobot();
    //grid can't be initialized if the robot is moving

    auto robotXWidth = std::stof(configparams->at("RobotXWidth").value);
    auto robotZLong = std::stof(configparams->at("RobotZLong").value);
    ROBOT_LENGTH = robotZLong;
    ROBOT_WIDTH = robotXWidth;
    robotBottomLeft     = QVec::vec3( - robotXWidth / 2 , 0, - robotZLong / 2);
    robotBottomRight    = QVec::vec3( + robotXWidth / 2 , 0, - robotZLong / 2);
    robotTopRight       = QVec::vec3( + robotXWidth / 2 , 0, + robotZLong / 2);
    robotTopLeft        = QVec::vec3( - robotXWidth / 2 , 0, + robotZLong / 2);
    robot_polygon_r  << QPointF(robotBottomLeft.x()-OFF, robotBottomLeft.z()-OFF)
                     << QPointF(robotBottomRight.x()+OFF,robotBottomRight.z()-OFF)
                     << QPointF(robotTopRight.x()+OFF,robotTopRight.z()+OFF)
                     << QPointF(robotTopLeft.x()-OFF,robotTopLeft.z()+OFF);

    collisions = std::make_shared<Collisions>();
    collisions->initialize(innerModel, configparams);
    grid.initialize(collisions, dim, false);
    grid.draw(scene);
    controller.initialize(innerModel, configparams, robot_polygon_r);

    //    reloj.restart();
};

template<typename TMap, typename TController>
void Navigation<TMap,TController>::update(const RoboCompLaser::TLaserData &laser_data, Target &target, bool needsReplaning)
{
    QVec current_robot_pose = innerModel->transformS6D("world", "robot");
    QVec nose_3d = innerModel->transform("world", QVec::vec3(0, 0, 250), "robot");
    const auto &[laser_poly, laser_cart] = read_laser_data(laser_data);
    QPolygonF current_robot_polygon = get_robot_polygon();
    QPointF current_robot_nose = QPointF(nose_3d.x(), nose_3d.z());

    if (needsReplaning)
    {
        pathPoints.clear();
        pathPoints = grid.computePath(current_robot_nose, target.pos);

        if (pathPoints.empty())
        {
            qDebug() << __FUNCTION__ << "Path not found. Returning";
            stopRobot();
            return;
        }
        needsReplaning = false;
    }
    compute_forces(pathPoints, laser_cart, laser_poly, current_robot_polygon, current_robot_nose);
    clean_points(pathPoints, laser_poly, current_robot_polygon);
    add_points(pathPoints, laser_poly, current_robot_polygon);
    draw_path(pathPoints, scene);
    auto[active, advVel, sideVel, rotVel] = controller.update(pathPoints,
                                                              laser_data, target.pos,
                                                              QPointF(current_robot_pose.x(),current_robot_pose.z()),
                                                              current_robot_nose);
    omnirobot_proxy->setSpeedBase(sideVel, advVel, rotVel);
};

template<typename TMap, typename TController>
void Navigation<TMap,TController>::stopRobot()
{
    qDebug() << "Navigation - " << __FUNCTION__;
    omnirobot_proxy->setSpeedBase(0, 0, 0);
}

template<typename TMap, typename TController>
void Navigation<TMap,TController>::compute_forces( std::vector<QPointF> &path,
                     const vector<QPointF> &laser_cart,
                     const QPolygonF &laser_poly,
                     const QPolygonF &current_robot_polygon,
                     const QPointF &current_robot_nose)
{
    if (path.size() < 3)
        return;

    // Go through points using a sliding window of 3
    for (auto &&[i, group] : iter::enumerate(iter::sliding_window(path, 3)))
    {
        if (group.size() < 3)
            continue; // break if too short

        auto p1 = QVector2D(group[0]);
        auto p2 = QVector2D(group[1]);
        auto p3 = QVector2D(group[2]);
        if (p1 == p2 or p2 == p3)
            continue;

        QPointF p = group[1];
        int index_of_p_in_path = i + 1;  //index of p in path

        ////////////////////////////////
        /// INTERNAL curvature forces on p2. Stretches the path locally
        /// Approximates the angle between adjacent segments: p2->p1, p2->p3
        ////////////////////////////////
        QVector2D iforce{0, 0};
        if (is_visible(p, laser_poly))
            iforce = ((p1 - p2) / (p1 - p2).length() + (p3 - p2) / (p3 - p2).length());

        ////////////////////////////////////////////
        /// External forces caused by obstacles repulsion field
        ///////////////////////////////////////////7
        float dist;
        QVector2D eforce;

        // compute forces from map on not visible points
        int nonVisiblePointsComputed = 0;
        if (not is_visible(p, laser_poly))
        {
            auto [obstacleFound, vectorForce] = grid.vectorToClosestObstacle(p);
            if (( not obstacleFound) or (nonVisiblePointsComputed > 10))
            {
                qDebug ()  << __FUNCTION__ << "No obstacles found in map for not visible point or it is more than 10 not visible points away";
                nonVisiblePointsComputed++;
                continue;
            }
            else
            {
                qDebug()  << __FUNCTION__  << "--- Obstacle found in grid ---";
                dist = vectorForce.length() - (ROBOT_LENGTH / 2);   // subtract robot semi-width

                std::clamp(dist, 0.01f, MAX_LASER_DIST);
                if (dist < MAX_DIST_TO_FORCE)  // if bigger no effect
                {
                    // rescale dist so 1 is ROBOT_LENGTH
                    float magnitude = (1.f / ROBOT_LENGTH) * dist;
                    // compute inverse square law
                    magnitude = 5.f / (magnitude * magnitude);
                    eforce = magnitude * vectorForce.normalized();
                }
            }
            nonVisiblePointsComputed++;
        }
       else   // compute forces from laser on visible point
       {
            // Compute the distances from all laser points to current point p2, trasforming them into forces and adding them up
            for (const auto &l: laser_cart)
            {
                // compute distance from laser measure to point minus RLENGTH/2 or 0 and keep it positive
                QVector2D vec = QVector2D(p) - QVector2D(l);
                float dist = vec.length() - (ROBOT_LENGTH / 2);
                std::clamp(dist, 0.01f, MAX_LASER_DIST);
                if (dist < MAX_DIST_TO_FORCE)  // if bigger no effect
                {
                    // rescale dist so 1 is ROBOT_LENGTH
                    float magnitude = (1.f / ROBOT_LENGTH) * dist;
                    // compute inverse square law
                    magnitude = 5.f / (magnitude * magnitude);
                    eforce = eforce + magnitude * vec.normalized();
                }
            };
        }

        // update node pos. KI and KE are approximating inverse Jacobians modules. This should be CHANGED
        // Directions are taken as the vector going from p to closest obstacle.
        auto total = (KI * iforce) + (KE * eforce);

        // Remove tangential component of repulsion force by projecting on line tangent to path (base_line)
        QVector2D base_line = (p1 - p3).normalized();
        const QVector2D itangential = QVector2D::dotProduct(total, base_line) * base_line;
        total = total - itangential;

        //
        // limiters CHECK!!!!!!!!!!!!!!!!!!!!!!
        float limit = 12;
        if (total.length() > limit)
            total = limit * total.normalized();
        if (total.length() < -limit)
            total = -limit * total.normalized();

        /// Compute additional restrictions to be forced in the minimization process
        // A) Check boundaries for final displacements
        // A.1) Move nodes only if it does not move inside objects
        // A.2) Does not move underneath the robot.
        // A.3) Does not exit the laser polygon
        QPointF temp_p = p + total.toPointF();
        //qInfo()  << __FUNCTION__  << "Total force "<< total.toPointF()<< " New Point "<< temp_p;
        //if (is_point_visitable(temp_p) and (not current_robot_polygon.containsPoint(temp_p, Qt::OddEvenFill))
            //and (std::none_of(std::begin(intimateSpaces), std::end(intimateSpaces),[temp_p](const auto &poly) { return poly.containsPoint(temp_p, Qt::OddEvenFill);}))
            //and (std::none_of(std::begin(personalSpaces), std::end(personalSpaces),[temp_p](const auto &poly) { return poly.containsPoint(temp_p, Qt::OddEvenFill);}))
        //        )
       // {
            path[index_of_p_in_path] = temp_p;
       // }
//            if( auto it = find_if(pathPoints.begin(), pathPoints.end(), [p] (auto & s){ return (s.x() == p.x() and s.y() == p.y() );}); it != pathPoints.end())
//            {
//                int index = std::distance(pathPoints.begin(), it);
//                pathPoints[index] = temp_p;
//            }
    }
    // Check if robot nose is inside the laser polygon
    if (is_visible(current_robot_nose, laser_poly))
        path[0] = current_robot_nose;
    else
        qWarning() << __FUNCTION__ << "Robot Nose not visible -- NEEDS REPLANNING ";
}

template<typename TMap, typename TController>
void Navigation<TMap,TController>::clean_points(std::vector<QPointF> &path, const QPolygonF &laser_poly, const QPolygonF &current_robot_polygon)
{
    qDebug() << __FUNCTION__;
    std::vector<QPointF> points_to_remove;
    for (const auto &group : iter::sliding_window(path, 2))
    {
        const auto &p1 = group[0];
        const auto &p2 = group[1];

        if (not is_visible(p1, laser_poly) or not is_visible(p2, laser_poly)) //not visible
            continue;

        if (p2 == path.back())
            break;
        // check if p1 was marked to erase in the previous iteration
        if (std::find(std::begin(points_to_remove), std::end(points_to_remove), p1) != std::end(points_to_remove))
            continue;

        float dist = QVector2D(p1 - p2).length();
        qDebug() << __FUNCTION__ << " dist <" << dist << 0.5 * ROAD_STEP_SEPARATION;
        if (dist < 0.5 * ROAD_STEP_SEPARATION)
            points_to_remove.push_back(p2);

        else if (current_robot_polygon.containsPoint(p2, Qt::OddEvenFill))
        {
            qDebug() << "-------------" << __FUNCTION__ << "------------- Removing point inside robot ";
            points_to_remove.push_back(p2);
        }
    }
    qDebug() << __FUNCTION__ << "Removed: " << points_to_remove.size();
    for (auto &&p : points_to_remove)
        path.erase(std::remove_if(path.begin(), path.end(), [p](auto &r) { return p == r; }), path.end());
}

template<typename TMap, typename TController>
void Navigation<TMap,TController>::add_points(std::vector<QPointF> &path, const QPolygonF &laser_poly, const QPolygonF &current_robot_polygon)
{
    // qDebug()<<"Navigation - "<< __FUNCTION__;
    std::vector<std::tuple<int, QPointF>> points_to_insert;
    for (auto &&[k, group] : iter::enumerate(iter::sliding_window(path, 2)))
    {
        auto &p1 = group[0];
        auto &p2 = group[1];

        if (not is_visible(p1, laser_poly) or not is_visible(p2, laser_poly)) //not visible
            continue;

        float dist = QVector2D(p1 - p2).length();
        qDebug() << __FUNCTION__ << " dist >" << dist << ROAD_STEP_SEPARATION;
        if (dist > ROAD_STEP_SEPARATION)
        {
            //Crucial que el punto se ponga mas cerca que la condición de entrada
            float l = 0.9 * ROAD_STEP_SEPARATION / dist;
            QLineF line(p1, p2);
            points_to_insert.emplace_back(k + 1, QPointF{line.pointAt(l)});
        }
    }
    qDebug() << __FUNCTION__ << "Added: " << points_to_insert.size();
    for (const auto &[l, p] : iter::enumerate(points_to_insert))
    {
        if (not current_robot_polygon.containsPoint(std::get<QPointF>(p), Qt::OddEvenFill))
            path.insert(path.begin() + std::get<int>(p) + l, std::get<QPointF>(p));
    }
}

/////////////////////////////////////////////////////////
template<typename TMap, typename TController>
std::tuple<QPolygonF, std::vector<QPointF>> Navigation<TMap,TController>::read_laser_data(const RoboCompLaser::TLaserData &laser_data)
{
    QPolygonF laser_poly;
    std::vector<QPointF> laser_cart;
    for (const auto l : laser_data)
    {
        //convert laser polar coordinates to cartesian
        float x = l.dist * sin(l.angle);
        float y = l.dist * cos(l.angle);
        QVec laserWorld = innerModel->transform("world", QVec::vec3(x, 0, y), "laser");
        laser_poly << QPointF(x, y);
        laser_cart.emplace_back(QPointF(laserWorld.x(), laserWorld.z()));
    }
    return std::make_tuple(laser_poly, laser_cart);
}

template<typename TMap, typename TController>
bool Navigation<TMap,TController>::is_point_visitable(QPointF point)
{
    return true;  //// NEEDS the GRID
}

template<typename TMap, typename TController>
bool Navigation<TMap,TController>::is_visible(QPointF p, const QPolygonF &laser_poly)
{
    QVec pointInLaser = innerModel->transform("laser", QVec::vec3(p.x(), 0.0, p.y()), "world");
    return laser_poly.containsPoint(QPointF(pointInLaser.x(), pointInLaser.z()), Qt::OddEvenFill);
}

template<typename TMap, typename TController>
QPolygonF Navigation<TMap,TController>::get_robot_polygon()
{
    QPolygonF robotP;
    auto bLWorld = innerModel->transform("world", robotBottomLeft, "base_mesh");
    auto bRWorld = innerModel->transform("world", robotBottomRight, "base_mesh");
    auto tRWorld = innerModel->transform("world", robotTopRight, "base_mesh");
    auto tLWorld = innerModel->transform("world", robotTopLeft, "base_mesh");
    robotP << QPointF(bLWorld.x(), bLWorld.z());
    robotP << QPointF(bRWorld.x(), bRWorld.z());
    robotP << QPointF(tRWorld.x(), tRWorld.z());
    robotP << QPointF(tLWorld.x(), tLWorld.z());
    return robotP;
}

////////// GRID RELATED METHODS //////////
template<typename TMap, typename TController>
void Navigation<TMap,TController>::updateFreeSpaceMap(bool drawGrid)
{
    qDebug() << "Navigation - " << __FUNCTION__;
    grid.resetGrid();

    //To set occupied
    for (auto &&poly_intimate : iter::chain(intimateSpaces, affordancesBlocked))
        grid.markAreaInGridAs(poly_intimate, false);

    for (auto[cost, polygonVec] : mapCostObjects)
    {
        for (auto polygon : polygonVec)
            grid.modifyCostInGrid(polygon, cost);
    }

    for (auto &&poly_soc : socialSpaces)
        grid.modifyCostInGrid(poly_soc, 8.0);

    for (auto &&poly_per : personalSpaces)
        grid.modifyCostInGrid(poly_per, 10.0);

}

template<typename TMap, typename TController>
void Navigation<TMap,TController>::draw_path(const std::vector<QPointF> &path, QGraphicsScene *scene)
{
    static std::vector<QGraphicsLineItem *> scene_road_points;
    ///////////////////////
    // Preconditions
    ///////////////////////
    if (path.size() == 0)
        return;

    /// clear previous points
    for (QGraphicsLineItem *item : scene_road_points)
        scene->removeItem((QGraphicsItem *) item);
    scene_road_points.clear();

    /// Draw all points
    QGraphicsLineItem *line1, *line2;
    std::string color;
    for (unsigned int i = 1; i < path.size(); i++)
        for (auto &&p_pair : iter::sliding_window(path, 2))
        {
            if (p_pair.size() < 2)
                continue;
            Eigen::Vector2d a_point(p_pair[0].x(), p_pair[0].y());
            Eigen::Vector2d b_point(p_pair[1].x(), p_pair[1].y());
            Eigen::Vector2d dir = a_point - b_point;
            Eigen::Vector2d dir_perp = dir.unitOrthogonal();
            Eigen::ParametrizedLine segment = Eigen::ParametrizedLine<double, 2>::Through(a_point, b_point);
            Eigen::ParametrizedLine<double, 2> segment_perp((a_point + b_point) / 2, dir_perp);
            auto left = segment_perp.pointAt(50);
            auto right = segment_perp.pointAt(-50);
            QLineF qsegment(QPointF(a_point.x(), a_point.y()), QPointF(b_point.x(), b_point.y()));
            QLineF qsegment_perp(QPointF(left.x(), left.y()), QPointF(right.x(), right.y()));

            if (i == 1 or i == path.size() - 1)
                color = "#00FF00"; //Green

            line1 = scene->addLine(qsegment, QPen(QBrush(QColor(QString::fromStdString(color))), 20));
            line2 = scene->addLine(qsegment_perp, QPen(QBrush(QColor(QString::fromStdString("#F0FF00"))), 20));

            line1->setZValue(2000);
            line2->setZValue(2000);
            scene_road_points.push_back(line1);
            scene_road_points.push_back(line2);
        }
}
