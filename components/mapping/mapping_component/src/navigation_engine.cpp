/*
 * Copyright (C) 2025 by YOUR NAME HERE
 */
#include "navigation_engine.h"
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/img/CImage.h>
#include <iostream>
#include <filesystem>

namespace fs = std::filesystem;

NavigationEngine::NavigationEngine()
{
    // Rutas por defecto (se crearán en la carpeta del binario)
    path_temp = "results/temp_mapping";
    path_final = "results/final_map";
}

NavigationEngine::~NavigationEngine()
{
    // Asegurar que si cerramos mientras mapeamos, se guarde el resultado
    if (currentMode == MODE_MAPPING) {
        finalizeMapping();
    }
}

void NavigationEngine::forceSave()
{
    if (currentMode == MODE_MAPPING) {
        std::cout << "[NavEngine] Forzando guardado de mapa..." << std::endl;
        saveMap(path_final, true); // Guardamos en la carpeta final
    } else {
        std::cout << "[NavEngine] Ignorado: No se puede guardar mapa estando en modo localización." << std::endl;
    }
}

void NavigationEngine::setMode(const std::string &modeStr, const mrpt::math::TPose2D &initialPose)
{
    if (modeStr == "localization") {
        currentMode = MODE_LOCALIZATION;
        std::cout << "[NavEngine] Iniciando Modo LOCALIZACIÓN (AMCL)" << std::endl;
        initLocalization();
    } else if(modeStr == "mapping"){
        currentMode = MODE_MAPPING;
        std::cout << "[NavEngine] Iniciando Modo MAPPING (ICP-SLAM)" << std::endl;

        initMapping();
    }
    else {
        currentMode = MODE_IDLE;
        std::cout << "[NavEngine] Waiting for setting mode" << std::endl;
    }
}

// =========================================================================
//                             MAPPING LOGIC
// =========================================================================

void NavigationEngine::initMapping()
{
    // 1. Limpieza de carpetas temporales
    if (fs::exists(path_temp)) fs::remove_all(path_temp);
    fs::create_directories(path_temp);

    // 2. Configurar Mapas (Grid + Puntos)
    mrpt::maps::TSetOfMetricMapInitializers mapInit;

    // GridMap (Para navegación y visualización)
    mrpt::maps::COccupancyGridMap2D::TMapDefinition defGrid;
    defGrid.resolution = 0.05f; // 5cm
    defGrid.insertionOpts.maxOccupancyUpdateCertainty = 0.8f;
    defGrid.insertionOpts.maxDistanceInsertion = 25.0f;

    // PointsMap (Necesario para el algoritmo ICP)
    mrpt::maps::CSimplePointsMap::TMapDefinition defPoints;
    defPoints.insertionOpts.minDistBetweenLaserPoints = 0.05f;

    mapInit.push_back(mrpt::maps::TMetricMapInitializer::Ptr(
        new mrpt::maps::COccupancyGridMap2D::TMapDefinition(defGrid)));
    mapInit.push_back(mrpt::maps::TMetricMapInitializer::Ptr(
        new mrpt::maps::CSimplePointsMap::TMapDefinition(defPoints)));

    // 3. Configurar Builder
    mapBuilder.ICP_options.mapInitializers = mapInit;
    
    // Tuning para evitar fallos de "Quality 0" en movimientos rápidos
    mapBuilder.ICP_params.thresholdDist = 0.60f;
    mapBuilder.ICP_params.thresholdAng = 7.0f * (M_PI / 180.0f);
    mapBuilder.ICP_options.minICPgoodnessToAccept = 0.15f; 
    mapBuilder.ICP_params.maxIterations = 80;

    mapBuilder.initialize();
}

void NavigationEngine::stepMapping(mrpt::obs::CObservation2DRangeScan::Ptr scan, const mrpt::poses::CPose2D &odom)
{
    // Crear Observación de Odometría para el SLAM
    auto obsOdom = mrpt::obs::CObservationOdometry::Create();
    obsOdom->odometry = odom;
    obsOdom->timestamp = scan->timestamp;
    obsOdom->hasEncodersInfo = true;

    mrpt::obs::CSensoryFrame sf;
    sf.insert(scan);
    sf.insert(obsOdom);

    // Acción vacía (se deduce de la odometría interna del SF)
    mrpt::obs::CActionCollection ac;
    mapBuilder.processActionObservation(ac, sf);

    // Auto-guardado periódico en carpeta temporal
    static int save_cnt = 0;
    if (save_cnt++ > 50) { 
        save_cnt = 0; 
        saveMap(path_temp, false); 
    }
}

void NavigationEngine::finalizeMapping()
{
    std::cout << "[NavEngine] Finalizando mapeo..." << std::endl;
    // Guardar última versión
    saveMap(path_temp, true);

    // Renombrar carpeta temporal a final
    if (fs::exists(path_final)) fs::remove_all(path_final);
    
    try {
        fs::rename(path_temp, path_final);
        std::cout << "[NavEngine] Mapa guardado definitivamente en: " << path_final << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "[NavEngine] Error moviendo carpeta final: " << e.what() << std::endl;
    }
}

void NavigationEngine::saveMap(const std::string &path, bool verbose)
{
    if (!fs::exists(path)) fs::create_directories(path);

    auto multi = mapBuilder.getCurrentlyBuiltMetricMap();
    if (multi.isEmpty()) return;

    if (auto grid = multi.mapByClass<mrpt::maps::COccupancyGridMap2D>())
    {
        // --- CORRECCIÓN: SERIALIZACIÓN MRPT 2.x ---
        // Usamos un Stream GZ (comprimido) y el Archive
        std::string fileName = path + "/mapa.gridmap";
        mrpt::io::CFileGZOutputStream f(fileName);

        if (f.fileOpenCorrectly()) {
            mrpt::serialization::archiveFrom(f) << *grid; // Serializamos el objeto
        }
        // ------------------------------------------
        
        // Guardar Imagen (.png)
        mrpt::img::CImage img;
        grid->getAsImage(img, true, true);
        img.saveToFile(path + "/visual.png");

        if (verbose) std::cout << "Mapa guardado en disco." << std::endl;
    }
}

// =========================================================================
//                             LOCALIZATION LOGIC
// =========================================================================

void NavigationEngine::initLocalization()
{
    std::string mapFile = path_final + "/mapa.gridmap";

    // Crear el objeto mapa vacío
    static_map = mrpt::maps::COccupancyGridMap2D::Create();

    // --- CORRECCIÓN: CARGA MRPT 2.x ---
    if (fs::exists(mapFile)) {
        mrpt::io::CFileGZInputStream f(mapFile);
        if (f.fileOpenCorrectly()) {
            mrpt::serialization::archiveFrom(f) >> *static_map; // Deserializamos
            std::cout << "[NavEngine] Mapa cargado correctamente." << std::endl;
        } else {
            std::cerr << "ERROR: No se puede abrir el archivo: " << mapFile << std::endl;
            return;
        }
    } else {
        std::cerr << "ERROR: No existe el archivo: " << mapFile << std::endl;
        std::cerr << "Ejecuta primero en modo 'mapping'." << std::endl;
        return;
    }
    // ----------------------------------

    // Configurar Filtro
    mrpt::bayes::CParticleFilter::TParticleFilterOptions pf_opts;
    pf_opts.adaptiveSampleSize = true;
    pf_opts.sampleSize = 500;
    pf_opts.max_loglikelihood_dyn_range = 15;
    mcl_pf.m_options = pf_opts;

    // Inicializar Partículas
    mcl_pdf.resetDeterministic(mrpt::math::TPose2D(0,0,0), 500);

    last_mcl_odom = mrpt::poses::CPose2D(0,0,0);
}

void NavigationEngine::stepLocalization(mrpt::obs::CObservation2DRangeScan::Ptr scan, const mrpt::poses::CPose2D &current_odom)
{
    if (!static_map) return;

    // 1. Calcular Incremento de Odometría (Acción)
    mrpt::poses::CPose2D odom_increment = current_odom - last_mcl_odom;
    last_mcl_odom = current_odom;

    // Configurar modelo de ruido de movimiento
    mrpt::obs::CActionRobotMovement2D odom_move;
    mrpt::obs::CActionRobotMovement2D::TMotionModelOptions odom_opts;
    odom_opts.modelSelection = mrpt::obs::CActionRobotMovement2D::mmGaussian;
    odom_opts.gaussianModel.a1 = 0.02f; // Ruido lineal (metros)
    odom_opts.gaussianModel.a3 = 0.05f; // Ruido angular (radianes)

    // Computar la distribución de movimiento
    odom_move.computeFromOdometry(odom_increment, odom_opts);

    // --------------------------------------------------------
    //    IMPLEMENTACIÓN MANUAL DEL FILTRO DE PARTÍCULAS (PF)
    // --------------------------------------------------------

    // El operador += aplica la acción probabilística a cada partícula
    // Esto mueve la nube según la odometría + ruido.
    size_t M = mcl_pdf.particlesCount();
    for (size_t i = 0; i < M; i++)
    {
        // 1. Generar una muestra aleatoria (ruido) según el modelo de movimiento
        mrpt::poses::CPose2D delta_noise;
        odom_move.drawSingleSample(delta_noise);

        // 2. Componer la pose antigua de la partícula con el movimiento
        // Convertimos a CPose2D para asegurar que la suma (+) maneje bien los ángulos (-PI, PI)
        mrpt::poses::CPose2D oldPose(mcl_pdf.m_particles[i].d);
        mrpt::poses::CPose2D newPose = oldPose + delta_noise;

        // 3. Guardar la nueva pose en la partícula (como TPose2D ligera)
        mcl_pdf.m_particles[i].d = newPose.asTPose();
    }
    // Recorremos cada partícula y comprobamos cuánto coincide el láser con el mapa
    // desde esa posición hipotética.

    // Optimizamos preparando el láser (necesario para likelihoods rápidos)
    // scan->buildAuxiliaryRangesIfNeeded(); // En MRPT moderno suele ser auto
	double sum_weights = 0.0; // Acumulador
    for (size_t i = 0; i < mcl_pdf.particlesCount(); i++)
    {
        auto &p = mcl_pdf.m_particles[i];

        // Convertimos TPose2D (ligera) a CPose2D (clase completa)
        mrpt::poses::CPose2D particlePose(p.d);

        // Calculamos probabilidad convirtiendo la pose a 3D
        double log_w = static_map->computeObservationLikelihood(
            *scan,
            mrpt::poses::CPose3D(particlePose) // Corrección
        );

        p.log_w += log_w;
		sum_weights += log_w;
    }
	if (mcl_pdf.particlesCount() > 0) {
        this->last_mean_likelihood = sum_weights / mcl_pdf.particlesCount();
    }
    // PASO C: NORMALIZACIÓN DE PESOS
    // Ajusta los pesos para que sumen 1 (es vital antes de re-muestrear)
    mcl_pdf.normalizeWeights();

    // PASO D: RE-MUESTREO (Resampling)
    // Si las partículas están muy degeneradas (poca variedad), creamos nuevas
    // concentradas donde hay más probabilidad.
    double ESS = mcl_pdf.ESS(); // Effective Sample Size
    if (ESS < mcl_pdf.particlesCount() * 0.5) // Umbral típico: 50%
    {
        // Usamos las opciones guardadas en mcl_pf (o unas nuevas)
        mrpt::bayes::CParticleFilter::TParticleFilterOptions pf_opts;
        pf_opts.adaptiveSampleSize = true;
        pf_opts.sampleSize = 500;

        mcl_pdf.performResampling(pf_opts);
    }
}

// Carga el mapa pero lo deja en "Standby"
bool NavigationEngine::loadMapForPreview(const std::string &mapPath)
{
    std::string mapFile = mapPath + "/mapa.gridmap";

    static_map = mrpt::maps::COccupancyGridMap2D::Create();

    if (fs::exists(mapFile)) {
        mrpt::io::CFileGZInputStream f(mapFile);
        if (f.fileOpenCorrectly()) {
            mrpt::serialization::archiveFrom(f) >> *static_map;
            return true;
        }
    }
    return false;
}

// Convierte coordenadas de imagen a mundo
bool NavigationEngine::pixelToMeters(int px, int py, float &out_x, float &out_y)
{
	if (!static_map) return false;

    // 1. Eje X: Directo (Izquierda es Izquierda)
    out_x = static_map->idx2x(px);
	out_y = static_map->idx2y(py);
    return true;
}

LocMetrics NavigationEngine::getMetrics()
{
    LocMetrics m;

    if (currentMode != MODE_LOCALIZATION) {
        return {0,0,0,0};
    }

    // --- CORRECCIÓN AQUÍ ---
    // En MRPT 2.x, la función devuelve un objeto, no modifica parámetros.
    // Usamos 'auto' para capturar la matriz de covarianza y la pose media automáticamente.

    auto [cov, meanPose] = mcl_pdf.getCovarianceAndMean();

    // -----------------------

    // Ahora 'cov' ya tiene datos reales.
    // La covarianza puede ser negativa infinitesimalmente por errores numéricos,
    // usamos abs() por seguridad antes de la raíz.
    m.std_x = std::sqrt(std::abs(cov(0,0)));
    m.std_y = std::sqrt(std::abs(cov(1,1)));
    m.std_phi = mrpt::RAD2DEG(std::sqrt(std::abs(cov(2,2))));
	m.quality = this->last_mean_likelihood;
    m.ess = mcl_pdf.ESS();

    return m;
}

void NavigationEngine::startLocalization(float x, float y, float phi_rad)
{
    if (!static_map) return;

    // Configurar Filtro
    mrpt::bayes::CParticleFilter::TParticleFilterOptions pf_opts;
    pf_opts.adaptiveSampleSize = true;
    pf_opts.sampleSize = 1000;
    mcl_pf.m_options = pf_opts;

    // Inicializar nube de partículas (Gaussiana alrededor del clic)
    mrpt::math::TPose2D initPose(x, y, phi_rad);

    // Usamos una covarianza inicial:
    // Poca incertidumbre en posición (el usuario ha hecho clic)
    // MUCHA incertidumbre en ángulo (PI), porque un clic no indica orientación.
    mrpt::poses::CPosePDFGaussian pdf;
    pdf.mean = mrpt::poses::CPose2D(initPose);

    mrpt::math::CMatrixDouble33 cov;
    cov.setIdentity();
    cov(0,0) = 0.25; // 0.5m de error X
    cov(1,1) = 0.25; // 0.5m de error Y
    cov(2,2) = M_PI; // 180 grados de error angular (Orientación desconocida)
    pdf.cov = cov;

    mcl_pdf.resetDeterministic(initPose, 1000);
    // O mejor, si tu versión de MRPT lo soporta: mcl_pdf.reset(pdf, 1000);
    // Si no, resetDeterministic pone todas en el punto, y el ruido de movimiento las dispersará luego.

    currentMode = MODE_LOCALIZATION; // ¡AHORA SÍ ACTIVAMOS LA LÓGICA!
    last_mcl_odom = mrpt::poses::CPose2D(0,0,0); // Reiniciar ref odom

    std::cout << "[NavEngine] Localización iniciada en: " << x << ", " << y << std::endl;
}

// =========================================================================
//                             PUBLIC INTERFACE
// =========================================================================

void NavigationEngine::processSensorData(mrpt::obs::CObservation2DRangeScan::Ptr scan, 
                                         const mrpt::poses::CPose2D &odomPose)
{
    if (currentMode == MODE_MAPPING) {
        stepMapping(scan, odomPose);
    } else {
        stepLocalization(scan, odomPose);
    }
}

mrpt::maps::COccupancyGridMap2D::Ptr NavigationEngine::getMapForViz()
{
    if (currentMode == MODE_MAPPING) {
        auto multi = mapBuilder.getCurrentlyBuiltMetricMap();
        if (not multi.isEmpty()) return multi.mapByClass<mrpt::maps::COccupancyGridMap2D>();
    } else {
        return static_map;
    }
    return nullptr;
}

mrpt::poses::CPose2D NavigationEngine::getCurrentRobotPose()
{
    if (currentMode == MODE_MAPPING) {
        return mrpt::poses::CPose2D(mapBuilder.getCurrentPoseEstimation()->getMeanVal());
    } else {
        return mcl_pdf.getMeanVal();
    }
}