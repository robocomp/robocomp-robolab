//
// Created by robolab on 24/01/20.
//

#include "controller.h"

void Controller::initialize(std::shared_ptr<InnerModel> innerModel_,
                            std::shared_ptr<RoboCompCommonBehavior::ParameterList> params_,
                            QPolygonF ext_robot_polygon_r)
{
    qDebug()<<"Controller - "<< __FUNCTION__;
    innerModel = innerModel_;
    this->time = QTime::currentTime();
    this->delay = delay*1000;	//msecs
    extended_robot_polygon_r = ext_robot_polygon_r;

    try
    {
        MAX_ADV_SPEED = QString::fromStdString(params_->at("MaxZSpeed").value).toFloat();
        MAX_ROT_SPEED = QString::fromStdString(params_->at("MaxRotationSpeed").value).toFloat();
        MAX_SIDE_SPEED = QString::fromStdString(params_->at("MaxXSpeed").value).toFloat();
        MAX_LAG = std::stof(params_->at("MinControllerPeriod").value);
        ROBOT_RADIUS_MM =  QString::fromStdString(params_->at("RobotRadius").value).toFloat();
        ROBOT_WIDTH_MM =  QString::fromStdString(params_->at("RobotXWidth").value).toFloat();
        ROBOT_LENGTH_MM =  QString::fromStdString(params_->at("RobotZLong").value).toFloat();

        qDebug()<< __FUNCTION__ << "CONTROLLER: Params from config:"  << MAX_ADV_SPEED << MAX_ROT_SPEED << MAX_SIDE_SPEED << MAX_LAG << ROBOT_RADIUS_MM;
    }
    catch (const std::out_of_range& oor)
    {   std::cerr << "CONTROLLER. Out of Range error reading parameters: " << oor.what() << '\n'; }
}

std::tuple<bool, float, float, float> Controller::update( const std::vector<QPointF> &points,
                                                          const RoboCompLaser::TLaserData &laser_data,
                                                          QPointF target, QPointF robot, QPointF robot_nose)
{
    //qDebug() << __FUNCTION__ << "------- Controller - ";
    if(points.size() < 2)
        return std::make_tuple(false, 0,0,0);

    float advVel = 0.f, sideVel = 0.f, rotVel = 0.f;

    // Compute euclidean distance to target
    float euc_dist_to_target = QVector2D(robot - target).length();
    auto is_increasing = [](float new_val)
    {
        static float ant_value = 0.f;
        bool res = false;
        if( new_val - ant_value > 0 ) res = true;
        ant_value = new_val;
        return res;
    };

    if ( (points.size() < 2) or (euc_dist_to_target < FINAL_DISTANCE_TO_TARGET)) /*or is_increasing(euc_dist_to_target))*/
    {
        qDebug() << __FUNCTION__;
        qDebug()<< "·························";
        qDebug()<< "···· TARGET ACHIEVED ····";
        qDebug()<< "·························";
        qDebug()<< " ";
        return std::make_tuple(false, 0, 0, 0);  //active, adv, side, rot
    }

    /// Compute rotational speed
    QLineF robot_to_nose(robot, robot_nose);
    float angle = -rewrapAngleRestricted(qDegreesToRadians(robot_to_nose.angleTo(QLineF(robot_nose, points[1])))); // WATCH SIGN
    if(angle >= 0)
        rotVel = std::clamp(angle, 0.f, MAX_ROT_SPEED);
    else
        rotVel = std::clamp(angle, -MAX_ROT_SPEED, 0.f);
//        if(euc_dist_to_target < 5*FINAL_DISTANCE_TO_TARGET)
//            rotVel = 0.f;
//        if( fabs(rotVel) < 0.01) rotVel = 0.f;

    std::cout << std::boolalpha << __FUNCTION__ << "Controller:  dist " << euc_dist_to_target << " der_dist  " << is_increasing(euc_dist_to_target) << " angle " << angle << " rotvel " << rotVel << std::endl;

    /// Compute advance speed
    std::min(advVel = MAX_ADV_SPEED * exponentialFunction(rotVel, 0.9, 0.1, 0), euc_dist_to_target);

    /// Compute bumper-away speed
    QVector2D total{0, 0};
    for (const auto &l : laser_data)
    {
        //float limit = (fabs(ROBOT_LENGTH_MM / 2.f * sin(l.angle)) + fabs(ROBOT_LENGTH_MM / 2.f * cos(l.angle))) + 200;
        QPointF lp(l.dist*sin(l.angle), l.dist*cos(l.angle));
        if(extended_robot_polygon_r.containsPoint(lp, Qt::OddEvenFill))
            total = total - QVector2D(lp);

        //float diff = limit - l.dist;
        //if (diff >= 0)
        //    total = total + QVector2D(-diff * cos(l.angle), -diff * sin(l.angle));
    }
    sideVel = total.x()/10;
    sideVel = std::clamp(sideVel, -MAX_SIDE_SPEED, MAX_SIDE_SPEED);
    std::cout << "-------" << std::endl;
    std::cout << std::boolalpha << __FUNCTION__ << "Controller output " << advVel << " " << sideVel << " " << rotVel  << std::endl;
    return std::make_tuple(true, advVel, sideVel, rotVel);
    //return std::make_tuple(true, 0,0,0);

}

// compute max de gauss(value) where gauss(x)=y  y min
float Controller::exponentialFunction(float value, float xValue, float yValue, float min)
{
    if (yValue <= 0)
        return 1.f;
    float landa = -fabs(xValue) / log(yValue);
    float res = exp(-fabs(value) / landa);
    return std::max(res, min);
}

float Controller::rewrapAngleRestricted(const float angle)
{
    if (angle > M_PI)
        return angle - M_PI * 2;
    else if (angle < -M_PI)
        return angle + M_PI * 2;
    else
        return angle;
}
