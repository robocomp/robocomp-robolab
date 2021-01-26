//
// Created by robolab on 17/01/20.
//

#include "navigation.h"
#include <QGraphicsLineItem>
#include <ranges>

template<typename TMap, typename TController>
void Navigation<TMap,TController>::initialize(  std::shared_ptr<InnerModel> innerModel_,
                                                std::shared_ptr<RoboCompCommonBehavior::ParameterList> configparams_,
                                                RoboCompOmniRobot::OmniRobotPrxPtr omnirobot_proxy_,
                                                QGraphicsScene *scene_, QPolygonF robot_polygon_)
{
    //qDebug() << "Navigation - " << __FUNCTION__;
    innerModel = innerModel_;
    configparams = configparams_;
    scene = scene_;
    omnirobot_proxy = omnirobot_proxy_;
    stopRobot();    //grid can't be initialized if the robot is moving

    robot_polygon_r = robot_polygon_;
    collisions = std::make_shared<Collisions>();
    collisions->initialize(innerModel_, configparams);
    grid.initialize(collisions, scene);
    //controller.initialize(innerModel, configparams, robot_polygon_r);
};

template<typename TMap, typename TController>
void Navigation<TMap,TController>::update( const RobotPose &robot_pose, const QPolygonF &laser_polygon, Target &target)
{
    static float previous_turn = 0;

    auto vector_arcos = compute_potential_set(robot_pose.y_vel, robot_pose.ang_vel);
    auto vector_sin_obs = remove_obstacle_hits(vector_arcos, laser_polygon, robot_polygon_r);
    QVec tr = innerModel->transform("robot", QVec::vec3(target.pos.x(), 0, target.pos.y()), "world");
    auto best_choice = cost_function(vector_sin_obs, tr.x(), tr.z(), robot_pose.x, robot_pose.y, previous_turn);

    if (best_choice.has_value())
    {
        auto[x, y, v, w, alpha] = best_choice.value();
        auto va = std::min(v / 5, 1000.f);
        try
        {
            omnirobot_proxy->setSpeedBase(0, va, -w);
            //omnirobot_proxy->setSpeedBase(0, 0, 0);
            previous_turn = -w;
        }  // w should come positive
        catch (const Ice::Exception &e)
        { std::cout << e.what() << std::endl; }
        vector_sin_obs.insert(vector_sin_obs.begin(), best_choice.value());
        draw_predictions(robot_pose, vector_sin_obs);
    }
};

/// Computes a navigation function for target_point on current grid
/// \param target_point
/// \return
template<typename TMap, typename TController>
bool Navigation<TMap, TController>::plan_new_path(QPointF target_point)
{
    if(auto target = grid.get_value(target_point); target.has_value())
    {
        grid.reset_cell_distances();
        float dist = 0;
        auto L1 = grid.neighboors(target.value(), dist++);
        std::vector<typename TMap::Value> L2;
        bool end = false;
        while (not end)
        {
            for (const auto &current_cell : L1)
            {
                auto selected = grid.neighboors(current_cell, dist);
                L2.insert(std::end(L2), std::begin(selected), std::end(selected));
            }
            dist++;
            end = L2.empty();
            L1.swap(L2);
            L2.clear();
        }
        return true;
    }
    else
        return false;
}

template<typename TMap, typename TController>
std::vector<std::vector<Tupla>> Navigation<TMap, TController>::compute_potential_set( float current_adv, float current_rot)
{
    std::vector <Tupla> vectorT;
    std::vector<std::vector<Tupla>> list_arcs;
    const float semiwidth = 50;
    // Compute future positions of the robot
    float dt = 2; // 1 second ahead
    for (float v = -100; v <= 700; v += 100) // advance speed
    {
        for (float w = -1; w <= 1; w += 0.1) //rotacion speed
        {
            std::vector<Tupla> list_points;
            float new_adv = current_adv + v;
            float new_rot = current_rot + w;
            if (fabs(w) > 0.01)
            {
                // Nuevo punto posible
                float r = new_adv / new_rot; // radio de giro ubicado en el eje x del robot
                float arc_length = new_rot * dt * r;
                for (float t = semiwidth; t < arc_length; t += semiwidth)
                    list_points.emplace_back(std::make_tuple(r - r * cos(t / r), r * sin(t / r), new_adv, new_rot, t / r));
            }
            else // para evitar la división por cero en el cálculo de r
            {
                for(float t = semiwidth; t < v*dt; t+=semiwidth)
                    list_points.emplace_back(std::make_tuple(0.f, t, new_adv, new_rot, new_rot*dt));
            }
            list_arcs.push_back(list_points);
        }
    }
    return list_arcs;
}
template<typename TMap, typename TController>
std::vector<Tupla> Navigation<TMap,TController>::remove_obstacle_hits(  const std::vector<std::vector<Tupla>> &vector_arcs,
                                                                        const QPolygonF &laser_polygon,
                                                                        const QPolygonF &robot_polygon_r)
{
    std::vector<Tupla> vector_obs;
    for(auto &arc_points : vector_arcs)
    {
        for (auto &point : arc_points)
        {
            auto [x, y, adv, giro, ang] = point;
            auto temp_robot = QTransform().translate(x,y).rotate(ang).map(robot_polygon_r);

            // the search ends if one point of the robot is outside of the laser polygon, thus the res will be different from std::end
            auto res = std::find_if_not(std::begin(temp_robot), std::end(temp_robot), [laser_polygon](const auto &p)
                        { return laser_polygon.containsPoint(p,Qt::OddEvenFill);});

            if(res == std::end(temp_robot))  //all inside
                vector_obs.emplace_back(point);
            else
                break;
        }
    }
    return vector_obs;
}

/**
 * Ordenamos cost function that combines several factors to select the best point in vector_points
 * @param vector_points
 * @param tx
 * @param ty
 * @return best speed configuration for the robot
 */
template<typename TMap, typename TController>
std::optional<Tupla> Navigation<TMap,TController>::cost_function( std::vector<Tupla> vector_points,
                                                                  float tx, float ty, float rx, float ry,
                                                                  float previous_turn)
{
    const float A=5, B=0.1, C=5;
    int k=0;
    std::vector<std::tuple<float, Tupla>> values;
    values.resize(vector_points.size());
    for(auto &point : vector_points)
    {
        auto [x, y, adv, giro, ang] = point;
        auto va = this->grid.get_value(x,y); auto vb = this->grid.get_value(x,y);
        if(va.has_value() and vb.has_value())
        {
            float nav_function = 1000;
            if(va.value().dist != -1)
                nav_function = va.value().dist;
            float dist_to_target = sqrt(pow(tx-x,2)+pow(ty-y,2));
            float dist_to_previous_turn =  fabs(-giro - previous_turn);
            //float dist_from_robot = 1/sqrt(pow(rx-x,2)+pow(ry-y,2));
            //float clearance_to_obstacle = 1/grid.dist_to_nearest_obstacle(x, y);
            values[k++] = std::make_tuple(A * nav_function + B* dist_to_target + C*dist_to_previous_turn, point);
        }
    }
    auto min = std::ranges::min_element(values, [](auto &a, auto &b){ return std::get<0>(a) < std::get<0>(b);});
    if(min != values.end())
        return std::get<Tupla>(*min);
    else
        return {};
}

/////////////////////////////////////////////////////////////////////////////////////////////////

template<typename TMap, typename TController>
void Navigation<TMap,TController>::stopRobot()
{
    qDebug() << "Navigation - " << __FUNCTION__;
    omnirobot_proxy->setSpeedBase(0, 0, 0);
}

/////////////////////////////////////////////////////////
/// draw future. Draw and arch going out from the robot
template<typename TMap, typename TController>
void Navigation<TMap,TController>::draw_predictions( const RobotPose &robot_pose, const std::vector <Tupla> &puntos)
{
    // remove existing arcspwd
    for (auto arc: arcs_vector)
        scene->removeItem(arc);
    arcs_vector.clear();

    QColor col("Blue");
    for (auto &[x, y, vx, wx, a] : puntos)
    {
        QPointF centro = robot_pose.robot_polygon_draw->mapToScene(x, y);
        arcs_vector.push_back(scene->addEllipse(centro.x(), centro.y(), 20, 20, QPen(col), QBrush(col)));
    }
    if(not puntos.empty())
    {
        auto front = puntos.front();
        QPointF selected = robot_pose.robot_polygon_draw->mapToScene(std::get<0>(front), std::get<1>(front));
        arcs_vector.push_back(scene->addEllipse(selected.x(), selected.y(), 80, 80, QPen(Qt::black), QBrush(Qt::black)));
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


