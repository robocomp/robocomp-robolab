//
// Created by robolab on 24/01/20.
//

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <CommonBehavior.h>
#include "genericworker.h"
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/range.hpp>
#include <tuple>
#include <innermodel/innermodel.h>

struct Edge_val{
	bool previous = false;
	bool current = false;
	void set (bool new_state)
	{
		previous = current;
		current = new_state;
	};
	bool rising_edge()
	{
		return (previous == false and current == true);
	};
	bool lowering_edge()
	{
		return (previous == true and current == false);
	};
};

class Controller
{
    public:
        void initialize( std::shared_ptr<InnerModel> innerModel_,
                         std::shared_ptr<RoboCompCommonBehavior::ParameterList> params_,
                         QPolygonF ext_robot_polygon_r);
        std::tuple<bool, float, float, float> update( const std::vector<QPointF> &points,
                                                      const RoboCompLaser::TLaserData &laser_data,
                                                      QPointF target, QPointF robot, QPointF robot_nose);


    private:
        std::shared_ptr<InnerModel> innerModel;
        QTime time;
        int delay;
        std::vector<float> baseOffsets;
        QPolygonF extended_robot_polygon_r;

        // Constants reassigned to the params values
        float MAX_ADV_SPEED;
        float MAX_ROT_SPEED;
        float MAX_SIDE_SPEED;
        float MAX_LAG; //ms
        float ROBOT_RADIUS_MM; //mm
        float ROBOT_WIDTH_MM;
        float ROBOT_LENGTH_MM;

        const float FINAL_DISTANCE_TO_TARGET = 250; //mm
        float KB = 3.0; // gain for bumper repulsive force

        // compute max de gauss(value) where gauss(x)=y  y min
        float exponentialFunction(float value, float xValue, float yValue, float min);
        float rewrapAngleRestricted(const float angle);
};

#endif //PROJECT_CONTROLLER_H
