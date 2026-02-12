#ifndef NAV_ENGINE_H
#define NAV_ENGINE_H

// --- INCLUDES NECESARIOS PARA SERIALIZACIÓN MRPT 2.x ---
#include <mrpt/io/CFileGZOutputStream.h> // Para guardar
#include <mrpt/io/CFileGZInputStream.h>  // Para cargar
#include <mrpt/serialization/CArchive.h> // Para el operador << y >>
// -------------------------------------------------------

#include <string>
#include <mrpt/slam/CMetricMapBuilderICP.h>
#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/math/TPose2D.h>

enum NavMode { MODE_MAPPING, MODE_LOCALIZATION, MODE_IDLE };

struct LocMetrics {
    double std_x;   // Desviación estándar en X (metros)
    double std_y;   // Desviación estándar en Y (metros)
    double std_phi; // Desviación estándar en Ángulo (grados)
    double ess;     // Effective Sample Size
	double quality;
};

class NavigationEngine {
public:
    NavigationEngine();
    ~NavigationEngine();


    // Fuerza el guardado del mapa actual a disco
    void forceSave();
	LocMetrics getMetrics();
    // Configuración
    void setMode(const std::string &modeStr, const mrpt::math::TPose2D &initialPose = {0,0,0});
    NavMode getMode() const { return currentMode; }

    // Ejecución principal
    void processSensorData(mrpt::obs::CObservation2DRangeScan::Ptr scan, 
                           const mrpt::poses::CPose2D &odomPose);

    // Obtener mapa para visualizar (Thread-safe logic inside MRPT)
    mrpt::maps::COccupancyGridMap2D::Ptr getMapForViz();
    mrpt::poses::CPose2D getCurrentRobotPose();

    // Fase 1: Carga el mapa pero NO arranca el AMCL todavía. Devuelve true si ok.
    bool loadMapForPreview(const std::string &mapPath);

    // Fase 2: Arranca AMCL en una posición (x, y) dada en metros
    void startLocalization(float x, float y, float phi_rad = 0.0f);

    // Helper para convertir píxeles a metros (necesario para el callback del ratón)
    // Devuelve true si la conversión es válida
    bool pixelToMeters(int px, int py, float &out_x, float &out_y);

private:
    NavMode currentMode;
    std::string path_temp, path_final;
	double last_mean_likelihood = 0.0; // Calidad del último ajuste (0 = Pésimo, 1 = Perfecto... en escala log es negativo)
    // --- MAPPING ---
    mrpt::slam::CMetricMapBuilderICP mapBuilder;
    void initMapping();
    void stepMapping(mrpt::obs::CObservation2DRangeScan::Ptr scan, const mrpt::poses::CPose2D &odom);
    void saveMap(const std::string &path, bool verbose);
    void finalizeMapping();

    // --- LOCALIZATION ---
    mrpt::poses::CPosePDFParticles mcl_pdf;
    mrpt::bayes::CParticleFilter mcl_pf;
    mrpt::maps::COccupancyGridMap2D::Ptr static_map;
    mrpt::poses::CPose2D last_mcl_odom; 

    void initLocalization();
    void stepLocalization(mrpt::obs::CObservation2DRangeScan::Ptr scan, const mrpt::poses::CPose2D &odom);


    // ...
};

#endif