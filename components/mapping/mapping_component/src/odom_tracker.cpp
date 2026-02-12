#include "odom_tracker.h"
#include <cmath>

OdomTracker::OdomTracker() {
    reset();
}

void OdomTracker::reset() {
    std::lock_guard<std::mutex> lock(mtx);
    acc_x = 0; acc_y = 0; acc_phi = 0;
    last_time = -1.0;
    history.clear();
}

void OdomTracker::processNewVelocity(const VelocityData &d) {
    std::lock_guard<std::mutex> lock(mtx);

    // Inicialización
    if (last_time < 0) {
        last_time = d.timestamp_seconds;
        history[d.timestamp_seconds] = mrpt::poses::CPose2D(0,0,0);
        return;
    }

    double dt = d.timestamp_seconds - last_time;
    if (dt <= 0.0001) return; // Evitar errores numéricos

    // Integración (Dead Reckoning)
    double dx = d.vx * dt;
    double dy = d.vy * dt;
    double dphi = d.w * dt;

    // Rotación al marco global
    double gx = dx * cos(acc_phi) - dy * sin(acc_phi);
    double gy = dx * sin(acc_phi) + dy * cos(acc_phi);

    acc_x += gx;
    acc_y += gy;
    acc_phi += dphi;

    // Normalizar ángulo (-PI, PI)
    while (acc_phi > M_PI) acc_phi -= 2*M_PI;
    while (acc_phi <= -M_PI) acc_phi += 2*M_PI;

    // Guardar
    history[d.timestamp_seconds] = mrpt::poses::CPose2D(acc_x, acc_y, acc_phi);
    last_time = d.timestamp_seconds;

    // Limpieza (Mantener solo últimos 5 segundos para ahorrar RAM)
    if (history.size() > 500) history.erase(history.begin());
}

bool OdomTracker::getPoseAtTime(double t, mrpt::poses::CPose2D &out) {
    std::lock_guard<std::mutex> lock(mtx);
    if (history.empty()) return false;

    auto it = history.upper_bound(t);
    
    if (it == history.begin()) { out = it->second; return true; }
    if (it == history.end())   { out = history.rbegin()->second; return true; }

    // Interpolación
    auto prev = std::prev(it);
    double t1 = prev->first;
    double t2 = it->first;
    double alpha = (t - t1) / (t2 - t1);

    const auto &p1 = prev->second;
    const auto &p2 = it->second;

    double x = p1.x() + alpha * (p2.x() - p1.x());
    double y = p1.y() + alpha * (p2.y() - p1.y());
    
    double ang_diff = p2.phi() - p1.phi();
    while (ang_diff <= -M_PI) ang_diff += 2*M_PI;
    while (ang_diff > M_PI) ang_diff -= 2*M_PI;
    double phi = p1.phi() + alpha * ang_diff;

    out = mrpt::poses::CPose2D(x, y, phi);
    return true;
}

mrpt::poses::CPose2D OdomTracker::getCurrentPose() {
    std::lock_guard<std::mutex> lock(mtx);
    return mrpt::poses::CPose2D(acc_x, acc_y, acc_phi);
}