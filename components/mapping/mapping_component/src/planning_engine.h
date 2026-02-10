//
// Created by robolab on 12/17/25.
//
#ifndef PLANNING_ENGINE_H
#define PLANNING_ENGINE_H

#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/nav/planners/PlannerSimple2D.h>
#include <vector>
#include <deque>

enum PlanState { PLAN_IDLE, PLAN_NAVIGATING, PLAN_ARRIVED, PLAN_FAILED };

class PlanningEngine {
public:
    PlanningEngine();
    mrpt::nav::PlannerSimple2D planner;
    // Configura el mapa base y genera internamente el mapa inflado (safety layer)
    void setMap(mrpt::maps::COccupancyGridMap2D::Ptr map, float robot_radius = 0.50);

    // Calcula la ruta A* desde start hasta target (en coordenadas de mundo)
    bool computePath(const mrpt::poses::CPose2D &start, const mrpt::poses::CPose2D &target);

    // Bucle de control: Devuelve v, w para seguir la ruta calculada
    bool getNavigationControl(const mrpt::poses::CPose2D &currentPose, double &out_v, double &out_w);

    // Getters
    PlanState getState() const { return state; }
    std::vector<mrpt::math::TPoint2D> getCurrentPath() const;
    mrpt::maps::COccupancyGridMap2D::Ptr getDilatedMap() { return dilatedMap; }

private:
    PlanState state = PLAN_IDLE;

    // Mapas
    mrpt::maps::COccupancyGridMap2D::Ptr originalMap;
    mrpt::maps::COccupancyGridMap2D::Ptr dilatedMap; // Mapa con obstáculos inflados
    mrpt::maps::COccupancyGridMap2D::Ptr tempMap;
    // Ruta
    std::deque<mrpt::math::TPoint2D> path;
    mrpt::math::TPoint2D currentTargetPoint;

    // Parámetros de Control Pure Pursuit
    double lookahead_dist = 0.6; // Distancia al punto objetivo local (m)
    double target_tolerance = 0.15; // Precisión de llegada (m)
    double max_v = 0.4; // m/s
    double max_w = 0.5; // rad/s

    // Placeholder para futura implementación reactiva
    bool getReactiveControl(const mrpt::poses::CPose2D &currentPose, double &out_v, double &out_w);

    // Implementación Pure Pursuit
    bool getPurePursuitControl(const mrpt::poses::CPose2D &currentPose, double &out_v, double &out_w);

    // Auxiliar: Inflar obstáculos según radio del robot
    void inflateObstacles(float radius_meters);
};

#endif