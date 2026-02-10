#ifndef ODOM_TRACKER_H
#define ODOM_TRACKER_H

#include <mutex>
#include <map>
#include <mrpt/poses/CPose2D.h>

// Estructura simple para no depender de RoboComp en este header si no quieres
struct VelocityData {
    double vx, vy, w;
    double timestamp_seconds;
};

class OdomTracker {
public:
    OdomTracker();
    
    // Procesa un nuevo dato de velocidad (Thread-safe)
    void processNewVelocity(const VelocityData &data);

    // Obtiene la pose interpolada en un instante t (Thread-safe)
    bool getPoseAtTime(double timestamp, mrpt::poses::CPose2D &out_pose);

    // Reinicia el tracker (útil al cambiar de modo)
    void reset();

    // Obtiene la última pose conocida
    mrpt::poses::CPose2D getCurrentPose();

private:
    std::mutex mtx;
    
    // Buffer histórico
    std::map<double, mrpt::poses::CPose2D> history;
    
    // Acumuladores (Dead Reckoning)
    double acc_x, acc_y, acc_phi;
    double last_time;
};

#endif