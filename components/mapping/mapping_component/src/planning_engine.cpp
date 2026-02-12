//
// Created by robolab on 12/17/25.
//
#include "planning_engine.h"
#include <mrpt/nav.h> // Cabecera general de navegación
#include <cmath>
#include <iostream>
#include <deque> // Necesario para std::deque

PlanningEngine::PlanningEngine() {
    state = PLAN_IDLE;
}

void PlanningEngine::setMap(mrpt::maps::COccupancyGridMap2D::Ptr map, float robot_radius) {
    if (!map) return;
    this->originalMap = map;

    // // // Crear copia para inflado
    // if (!this->dilatedMap) {
    //     this->dilatedMap = mrpt::maps::COccupancyGridMap2D::Create();
    // }
    // *this->dilatedMap = *originalMap;

    // Nota: No configuramos el planner aquí porque PlannerSimple2D no guarda estado.
    // Le pasaremos el mapa directamente al calcular.

    std::cout << "[Planning] Generando mapa de costes (Inflation r=" << robot_radius << "m)..." << std::endl;
    // inflateObstacles(robot_radius);
}

void PlanningEngine::inflateObstacles(float radius_meters) {
    // Algoritmo simple de dilatación morfológica
    if (!dilatedMap) return;

    float resolution = dilatedMap->getResolution();
    int radius_cells = std::ceil(radius_meters / resolution);

    int size_x = dilatedMap->getSizeX();
    int size_y = dilatedMap->getSizeY();

    // Copia temporal
    if (!this->tempMap) {
        this->tempMap = mrpt::maps::COccupancyGridMap2D::Create();
    }
    *this->tempMap = *dilatedMap;

    // Iteramos todo el mapa
    for (int x = 0; x < size_x; x++) {
        for (int y = 0; y < size_y; y++) {
            // > 0.49f significa ocupado (probabilidad > 0.5)
            if (tempMap->getCell(x, y) > 0.49f) {
                // Pintar un círculo alrededor
                for (int dx = -radius_cells; dx <= radius_cells; dx++) {
                    for (int dy = -radius_cells; dy <= radius_cells; dy++) {
                        if (dx*dx + dy*dy <= radius_cells*radius_cells) {
                            int nx = x + dx;
                            int ny = y + dy;
                            if (nx >= 0 && nx < size_x && ny >= 0 && ny < size_y) {
                                dilatedMap->setCell(nx, ny, 1.0f); // Obstáculo
                            }
                        }
                    }
                }
            }
        }
    }
    std::cout << "[Planning] Mapa inflado listo." << std::endl;
}

bool PlanningEngine::computePath(const mrpt::poses::CPose2D &start, const mrpt::poses::CPose2D &target) {
    // if (!dilatedMap) {
    //     std::cerr << "[Planning] ERROR: No hay mapa cargado." << std::endl;
    //     return false;
    // }
    //
    // // 1. Verificar si el destino está libre
    // float occ = dilatedMap->getPos(target.x(), target.y());
    // if (occ > 0.55f) {
    //     std::cerr << "[Planning] ERROR: El destino está en un obstáculo." << std::endl;
    //     state = PLAN_FAILED;
    //     return false;
    // }

    // 2. Preparar variables
    std::deque<mrpt::math::TPoint2D> calculatedPath;

    // CAMBIO 1: El 5º argumento es un bool de salida "notFound"
    bool notFound = false;

    mrpt::poses::CPose2D origin(start.x(), start.y(), 0.0);
    mrpt::poses::CPose2D destination(target.x(), target.y(), 0.0);

    std::cout << "[Planning] Calculando ruta desde (" << origin.x() << ", " << origin.y() << ") a ("
              << destination.x() << ", " << destination.y() << ")..." << std::endl;

    // 3. Calcular
    // Nota: Esta función devuelve void, el resultado se ve en 'notFound' y 'calculatedPath'
    // Fix: call with the shared_ptr map and the original const refs

    planner.minStepInReturnedPath = 0.2f;
    planner.robotRadius = 0.6f;
    planner.occupancyThreshold = 0.5f;

    planner.computePath(
        *originalMap,      // const mrpt::maps::COccupancyGridMap2D&
        origin,           // const mrpt::poses::CPose2D&
        destination,      // const mrpt::poses::CPose2D&
        calculatedPath,   // std::deque<mrpt::math::TPoint2D>&
        notFound,         // bool&
        -1.0f             // float maxSearchPathLength (-1 = no limit)
    );

    // Verificamos si falló usando el flag o si el path está vacío
    if (notFound || calculatedPath.empty()) {
        std::cerr << "[Planning] No se encontró ruta." << std::endl;
        state = PLAN_FAILED;
        return false;
    }

    // 4. Guardar ruta
    this->path = calculatedPath;

    if (!this->path.empty()) this->path.pop_front();

    state = PLAN_NAVIGATING;
    std::cout << "[Planning] Ruta calculada: " << this->path.size() << " puntos." << std::endl;
    return true;
}

std::vector<mrpt::math::TPoint2D> PlanningEngine::getCurrentPath() const {
    std::vector<mrpt::math::TPoint2D> v;
    for(const auto &p : path) v.push_back(p);
    return v;
}

bool PlanningEngine::getNavigationControl(const mrpt::poses::CPose2D &currentPose, double &out_v, double &out_w) {
    if (state != PLAN_NAVIGATING) {
        out_v = 0; out_w = 0;
        return false;
    }
    return getPurePursuitControl(currentPose, out_v, out_w);
}

// ======================= PURE PURSUIT =======================

// bool PlanningEngine::getPurePursuitControl(const mrpt::poses::CPose2D &robotPose, double &out_v, double &out_w) {
//     if (path.empty()) {
//         state = PLAN_ARRIVED;
//         out_v = 0; out_w = 0;
//         std::cout << "[Planning] META ALCANZADA." << std::endl;
//         return false;
//     }

//     // Definir constantes locales (o moverlas al .h)
//     const double target_tolerance = 0.15; // 15cm
//     const double lookahead_dist = 0.60;   // 60cm
//     const double max_v_limit = 0.6;
//     const double max_w_limit = 0.4;

//     // 1. Encontrar el punto objetivo (Lookahead)
//     mrpt::math::TPoint2D goal;

//     while (!path.empty()) {
//         goal = path.front();

//         // Calcular distancia manualmente
//         double dx_g = goal.x - robotPose.x();
//         double dy_g = goal.y - robotPose.y();
//         double dist = std::sqrt(dx_g*dx_g + dy_g*dy_g);

//         if (dist < target_tolerance && path.size() == 1) {
//             // Fin de ruta
//             path.clear();
//             state = PLAN_ARRIVED;
//             out_v = 0; out_w = 0;
//             return false;
//         }

//         if (dist < lookahead_dist && path.size() > 1) {
//             // Punto superado, vamos al siguiente
//             path.pop_front();
//             continue;
//         }
//         break; // Goal válido encontrado
//     }

//     // 2. Transformar goal al marco local
//     double dx = goal.x - robotPose.x();
//     double dy = goal.y - robotPose.y();

//     double lx = dx * cos(robotPose.phi()) + dy * sin(robotPose.phi());
//     double ly = -dx * sin(robotPose.phi()) + dy * cos(robotPose.phi());

//     // 3. Calcular Curvatura
//     double L2 = lx*lx + ly*ly;
//     if (L2 < 0.001) { out_v=0; out_w=0; return true; }

//     double gamma = (2.0 * ly) / L2;

//     // 4. Calcular velocidades
//     out_v = max_v_limit;
//     out_w = gamma * out_v;

//     if (out_w > max_w_limit) out_w = max_w_limit;
//     if (out_w < -max_w_limit) out_w = -max_w_limit;

//     if (std::abs(out_w) > max_w_limit * 0.5) out_v *= 0.5;

//     return true;
// }

bool PlanningEngine::getPurePursuitControl(
    const mrpt::poses::CPose2D &robotPose,
    double &out_v,
    double &out_w)
{
    if (path.empty()) {
        state = PLAN_ARRIVED;
        out_v = 0.0;
        out_w = 0.0;
        std::cout << "[Planning] META ALCANZADA." << std::endl;
        return false;
    }

    // =========================
    // Parámetros de control
    // =========================
    const double target_tolerance = 0.25;   // m
    const double lookahead_dist   = 0.60;   // m

    const double max_v_limit = 0.70;        // m/s
    const double max_w_limit = 0.5;        // rad/s

    // Sigmoide de avance
    const double theta_0    = 0.5;         // rad (~20º)
    const double k_sigmoid = 100.0;          // pendiente

    // =========================
    // 1. Selección del punto objetivo
    // =========================
    mrpt::math::TPoint2D goal;

    while (!path.empty()) {
        goal = path.front();

        double dx_g = goal.x - robotPose.x();
        double dy_g = goal.y - robotPose.y();
        double dist = std::sqrt(dx_g*dx_g + dy_g*dy_g);

        if (dist < target_tolerance && path.size() == 1) {
            path.clear();
            state = PLAN_ARRIVED;
            out_v = 0.0;
            out_w = 0.0;
            return false;
        }

        if (dist < lookahead_dist && path.size() > 1) {
            path.pop_front();
            continue;
        }
        break;
    }

    // =========================
    // 2. Transformar objetivo a marco local
    // =========================
    double dx = goal.x - robotPose.x();
    double dy = goal.y - robotPose.y();

    double cos_th = std::cos(robotPose.phi());
    double sin_th = std::sin(robotPose.phi());

    double lx =  dx * cos_th + dy * sin_th;
    double ly = -dx * sin_th + dy * cos_th;

    // =========================
    // 3. Error angular
    // =========================
    double heading_error = std::atan2(ly, lx);  // [-pi, pi]

    // =========================
    // 4. Curvatura Pure Pursuit
    // =========================
    double L2 = lx*lx + ly*ly;
    if (L2 < 1e-3) {
        out_v = 0.0;
        out_w = 0.0;
        return true;
    }

    double gamma = (2.0 * ly) / L2;

    // =========================
    // 5. Velocidad angular
    // =========================
    out_w = gamma * max_v_limit;

    if (out_w >  max_w_limit) out_w =  max_w_limit;
    if (out_w < -max_w_limit) out_w = -max_w_limit;

    // =========================
    // 6. Control sigmoidal de velocidad lineal
    // =========================

    // Objetivo detrás → rotación pura
    if (lx < 0.0) {
        out_v = 0.0;
        return true;
    }

    // Sigmoide en función del error angular
    double abs_heading = std::abs(heading_error);
    double sigmoid = 1.0 / (1.0 + std::exp(k_sigmoid * (abs_heading - theta_0)));

    out_v = max_v_limit * sigmoid;

    // Seguridad adicional: si el giro es fuerte, frenar más
    double w_ratio = std::abs(out_w) / max_w_limit;
    out_v *= (1.0 - w_ratio);

    if (out_v < 0.0)
        out_v = 0.0;

    return true;
}