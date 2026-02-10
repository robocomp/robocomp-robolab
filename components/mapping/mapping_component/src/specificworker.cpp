/*
 * Copyright (C) 2025 by YOUR NAME HERE
 */
#include "specificworker.h"

// IMPORTANT: Assuming you have this file from the previous explanation
#include "odom_tracker.h"

// Variable global auxiliar (OpenCV C-style callbacks son un dolor con clases C++)
static SpecificWorker* g_worker = nullptr;

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
	g_worker = this;
    initTerminal(); // <--- INICIAR MODO RAW
    std::cout << "=== CONTROL POR TECLADO ACTIVADO ===" << std::endl;
    std::cout << " [S] Guardar Mapa" << std::endl;
    std::cout << " [L] Cambiar a Localizacion" << std::endl;
    std::cout << " [M] Cambiar a Mapping" << std::endl;
    std::cout << "====================================" << std::endl;

	this->startup_check_flag = startup_check;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif

		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0){
			qWarning() << error;
			throw error;
		}
	}
}

SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	//G->write_to_json_file("./"+agent_name+".json");
}

void SpecificWorker::onMouse(int event, int x, int y, int flags, void* userdata)
{
    SpecificWorker* worker = static_cast<SpecificWorker*>(userdata);
    if (!worker) worker = g_worker; // fallback if userdata was null

    if (worker) {
        worker->handleMouse(event, x, y, flags);
    }
}

void SpecificWorker::handleMouse(int event, int x, int y, int flags)
{

    // CASO 2: DEFINIR OBJETIVO DE RUTA (Nueva lógica)
    if (waitingForTarget && event == cv::EVENT_LBUTTONDOWN)
    {
        qInfo() << "Waiting";
        float tx, ty;
        if (navEngine.pixelToMeters(x, y, tx, ty)) {
            std::cout << "[MOUSE] Objetivo marcado en: " << tx << ", " << ty << std::endl;

            // Obtenemos pose actual del robot
            mrpt::poses::CPose2D robotPose = navEngine.getCurrentRobotPose();
            mrpt::poses::CPose2D targetPose(tx, ty, 0);

            // Calculamos ruta
            if (planner.computePath(robotPose, targetPose)) {
                std::cout << "[Planner] Ruta OK. Iniciando movimiento..." << std::endl;
            } else {
                std::cout << "[Planner] Fallo al calcular ruta." << std::endl;
            }

            waitingForTarget = false; // Dejamos de esperar clic
        }
    }

    if (!waitingForInitialPose) return; // Solo si estamos en modo "Set Pose"

    if (event == cv::EVENT_LBUTTONDOWN)
    {

        // 1. Empezamos a arrastrar
        isDragging = true;
        dragStartX = x;
        dragStartY = y;
        dragEndX = x;
        dragEndY = y;
    }
    else if (event == cv::EVENT_MOUSEMOVE && isDragging)
    {
        // 2. Actualizamos la línea visual
        dragEndX = x;
        dragEndY = y;
    }
    else if (event == cv::EVENT_LBUTTONUP && isDragging)
    {
        // 3. Soltamos: Calcular ángulo y arrancar
        isDragging = false;


        // 1. Obtenemos coordenadas EN METROS (usando la función corregida arriba)
        // Nota: Al usar pixelToMeters, startY_m y endY_m YA TIENEN la Y correcta (mundo).
        float startX_m, startY_m, endX_m, endY_m;
        navEngine.pixelToMeters(dragStartX, dragStartY, startX_m, startY_m);
        navEngine.pixelToMeters(x, y, endX_m, endY_m);

        // 2. Calculamos diferencias en el mundo real
        float d_x_world = endX_m - startX_m;
        float d_y_world = endY_m - startY_m;

        // 3. Calculamos el ángulo
        // Como ya estamos en coordenadas de mundo corregidas, el atan2 estándar funciona.
        float angle_rad = atan2(d_y_world, d_x_world);

        // --- VERIFICACIÓN DE SIMETRÍA ---
        // Si a pesar de esto te sale el ángulo al revés, descomenta la siguiente línea.
        // (Esto pasa si tu pixelToMeters no invierte, pero como lo hemos arreglado en el paso 1,
        //  prueba primero SIN el menos).

        // angle_rad = -angle_rad;

        std::cout << "[MOUSE] Pose: " << startX_m << ", " << startY_m
                  << " | Angulo: " << (angle_rad * 180.0/M_PI) << " deg" << std::endl;

        navEngine.startLocalization(startX_m, startY_m, angle_rad);
        waitingForInitialPose = false;

    }

}

void SpecificWorker::initialize()
{
    std::cout << "[SpecificWorker] Inicializando..." << std::endl;

    // 1. Leer configuración de modo (mapping vs localization)
    // El valor por defecto es 'mapping' si no existe en config
    std::string mode = "mapping";

    // 2. Configurar el Motor de Navegación
    navEngine.setMode(mode);
}

void SpecificWorker::compute()
{
    if (kbhit())
    {
        char key = getch();


        if (key == 'p') // Planning
        {
            if (navEngine.getMode() != MODE_LOCALIZATION)
            {
                std::cout << "[KEY] ERROR: Debes estar localizado para planificar." << std::endl;
            } else
            {
                std::cout << "\n--------------------------------------------------" << std::endl;
                std::cout << " MODO PLANIFICACION ACTIVADO" << std::endl;
                std::cout << " Haz clic en el mapa para marcar el OBJETIVO." << std::endl;
                std::cout << "--------------------------------------------------" << std::endl;

                // Pasamos el mapa actual al planner (con inflado de 0.4m)
                // Nota: navEngine.getMapForViz() devuelve el grid estático si estamos en localización
                planner.setMap(navEngine.getMapForViz(), 0.6f);

                waitingForTarget = true;
                waitingForInitialPose = false; // Por seguridad
            }
        }
        // Convertir a minúscula para facilitar
        if (key >= 'A' && key <= 'Z') key += 32;
        else if (key == 'r') // Relocalize / Reset Pose
        {
            if (navEngine.getMode() != MODE_LOCALIZATION)
            {
                std::cout << "\n[KEY] RECOLOCACION MANUAL SOLICITADA." << std::endl;
                std::cout << "Haz clic y arrastra en el mapa para corregir la pose." << std::endl;
                waitingForInitialPose = true;
                // No llamamos a loadMapForPreview, solo activamos la espera de clic
            }
        }
        if (key == 's')
        {
            std::cout << "\n[KEY] ¡Guardando mapa!" << std::endl;
            navEngine.forceSave();
        } else if (key == 'l')
        {
            // CAMBIO: Lógica de selección de mapa y espera de clic
            std::string mapPath = askUserForMap(); // Función auxiliar (ver abajo)

            if (!mapPath.empty())
            {
                // Cargamos mapa en modo 'Preview' (sin arrancar AMCL todavía)
                if (navEngine.loadMapForPreview(mapPath))
                {
                    std::cout << "\n--------------------------------------------------" << std::endl;
                    std::cout << " MAPA CARGADO: " << mapPath << std::endl;
                    std::cout << " AHORA HAZ CLIC EN LA VENTANA 'Navegacion'" << std::endl;
                    std::cout << " PARA INDICAR LA POSICION INICIAL DEL ROBOT." << std::endl;
                    std::cout << "--------------------------------------------------" << std::endl;
                    // Calcular centro de la pantalla
                    int screenW = 1000;
                    int screenH = 800;

                    // Calcular tamaño del mapa cargado
                    int mapW = navEngine.getMapForViz()->getSizeX(); // o rawImg.cols
                    int mapH = navEngine.getMapForViz()->getSizeY(); // o rawImg.rows

                    // Centrar
                    view_scale = 1.0f; // Zoom inicial
                    view_pan_x = (screenW - mapW * view_scale) / 2.0f;
                    view_pan_y = (screenH - mapH * view_scale) / 2.0f;
                    waitingForInitialPose = true;
                    // Nota: No cambiamos currentMode a LOCALIZATION todavía
                }
            } else
            {
                std::cout << "[KEY] Selección cancelada." << std::endl;
            }
        } else if (key == 'm')
        {
            if (navEngine.getMode() != MODE_MAPPING)
            {
                std::cout << "\n[KEY] Reiniciando MODO MAPPING..." << std::endl;
                navEngine.setMode("mapping");
            }
        }
    }
    try
    {
        // -----------------------------------------------------------
        // 1. ADQUISICIÓN DE DATOS (Lidar + Odometría Sincronizada)
        // -----------------------------------------------------------

        auto data = lidar3d_proxy->getLidarDataWithThreshold2d("helios", 30000, 1);

        // Convertir tiempo a Segundos (double)
        double t_lidar = (double) data.timestamp / 1000.0;
        if (t_lidar == 0) t_lidar = mrpt::Clock::nowDouble();

        // Pedir al Tracker dónde estaba el robot EXACTAMENTE en t_lidar
        mrpt::poses::CPose2D odomPose;
        if (!odomTracker.getPoseAtTime(t_lidar, odomPose))
        {
            // Aún no tenemos suficiente historia de odometría
            return;
        }

        // -----------------------------------------------------------
        // 2. CONVERSIÓN DE SENSORES (RoboComp -> MRPT)
        // -----------------------------------------------------------

        auto obsScan = mrpt::obs::CObservation2DRangeScan::Create();
        obsScan->aperture = 2.0 * M_PI;
        obsScan->maxRange = 10.0;
        obsScan->sensorPose = mrpt::poses::CPose3D(0, 0, 0);
        obsScan->timestamp = mrpt::Clock::fromDouble(t_lidar);
        obsScan->rightToLeft = true;

        // Conversión mm -> m y Cartesianas -> Polares
        std::vector<float> ranges(360, 0.0f);
        std::vector<char> valid(360, 0);

        for (const auto &p: data.points)
        {
            if (p.z > 2000)
                continue;

            float user_x_mm = p.x;
            float user_y_mm = p.y;

            // --- CORRECCIÓN DE ESPEJO ---
            // Tu Y (Adelante) -> MRPT X (Adelante)
            float x_m = user_y_mm * 0.001f;

            // Tu X (Derecha) -> MRPT Y (Izquierda)
            // ANTES: y_m = -user_x_mm ...
            // AHORA: y_m = user_x_mm ... (Positivo)
            float y_m = user_x_mm * 0.001f;
            // ----------------------------

            float range = sqrt(x_m * x_m + y_m * y_m);
            float angle = atan2(y_m, x_m);

            // Indice 0..359
            int idx = round(((angle + M_PI) / (2 * M_PI)) * 360.0);
            if (idx >= 0 && idx < 360)
            {
                if (valid[idx] == 0 || range < ranges[idx])
                {
                    ranges[idx] = range;
                    valid[idx] = 1;
                }
            }
        }

        obsScan->resizeScan(360);
        for (int i = 0; i < 360; i++)
        {
            obsScan->setScanRange(i, valid[i] ? ranges[i] : obsScan->maxRange + 1.0f);
            obsScan->setScanRangeValidity(i, valid[i]);
        }

        // -----------------------------------------------------------
        // 3. EJECUCIÓN DE NAVEGACIÓN
        // -----------------------------------------------------------

        // Delegamos al engine. Él decide si mapea o localiza.
        // CAMBIO CRÍTICO: Solo ejecutamos el motor si NO estamos esperando el clic.
        if (!waitingForInitialPose)
        {
            navEngine.processSensorData(obsScan, odomPose);
        }

        // -----------------------------------------------------------
        // CONTROL DE NAVEGACION
        // -----------------------------------------------------------
        if (planner.getState() == PLAN_NAVIGATING)
        {
            double v = 0, w = 0;
            mrpt::poses::CPose2D currentPose = navEngine.getCurrentRobotPose();

            // Obtenemos comandos del Pure Pursuit
            bool keepGoing = planner.getNavigationControl(currentPose, v, w);

            if (keepGoing)
            {
                // ENVIAR A ROBOCOMP (Asegúrate de tener el proxy configurado)
                // differentialrobot_proxy->setSpeedBase(v, w);

                // Conversión a velocidades omnirobot
                float advx = 0.0f; // Avance en X
                float advz = v * 1000; // Avance en Z (lateral
                float rot = w; // Rotación

                // this->omnirobot_proxy->setSpeedBase(advx, advz, rot);
                this->differentialrobot_proxy->setSpeedBase(advz, -rot);
                std::cout << "CMD -> v: " << v << " w: " << w << "\r";
            } else
            {
                // Hemos llegado o fallado -> Parar robot
                // this->omnirobot_proxy->setSpeedBase(0, 0, 0);
                this->differentialrobot_proxy->setSpeedBase(0.0, 0.0);
                std::cout << "\n[Planner] Navegacion finalizada." << std::endl;
            }
        }
        // -----------------------------------------------------------
        // 4. VISUALIZACIÓN (OpenCV)
        // -----------------------------------------------------------
        updateVis();
    } catch (const std::exception &e)
    {
        std::cerr << "[SpecificWorker] Exception: " << e.what() << std::endl;
    }
}

void SpecificWorker::updateVis()
{
    // 1. Obtener referencia al mapa
    auto grid = navEngine.getMapForViz();
    if (!grid) return;

    mrpt::poses::CPose2D robotPose = navEngine.getCurrentRobotPose();

	// qInfo() << "Robot Pose (grid) (m): " << robotPose.x() << ", " << robotPose.y() << ", " << robotPose.phi();

    // 2. Generar imagen (Con vertical flip = true para que se vea bien el mapa)
    mrpt::img::CImage img;
    grid->getAsImage(img, true, true);
    cv::Mat cvImg = img.asCvMat<cv::Mat>(mrpt::img::SHALLOW_COPY);

    // -----------------------------------------------------------
    // CORRECCIÓN DE COORDENADAS (ROBOT -> PÍXELES)
    // -----------------------------------------------------------

	// --- COORDENADAS ESTÁNDAR ---
    // Como la imagen tiene flip vertical, el pixel 0 está arriba.
    // El grid tiene 0 abajo.
    // Fórmula estándar: H - 1 - idx

    int px = grid->x2idx(robotPose.x());
    int py = grid->y2idx(robotPose.y());

    // --- FLECHA ESTÁNDAR ---
    float arrowLen = 20.0;

    // En imágenes, Y crece hacia abajo.
    // Un ángulo positivo (CCW) debe hacer que la Y disminuya (suba en pantalla).
    // Por tanto: -sin() es lo correcto.
    int px_a = px + cos(robotPose.phi()) * arrowLen;
    int py_a = py + sin(robotPose.phi()) * arrowLen;
	//qInfo() << "Robot Pose init point (m): " << px << ", " << py ;
	//qInfo() << "Robot Pose final point (m): " << px_a << ", " << py_a ;
    // -----------------------------------------------------------
    // DIBUJADO
    // -----------------------------------------------------------

	// -----------------------------------------------------------
    // 3. APLICAR ZOOM Y PAN (CANVAS DINÁMICO)
    // -----------------------------------------------------------

    // Dibujamos instrucciones si estamos esperando clic
	if (waitingForInitialPose)
    {
        cv::putText(cvImg, "ARRASTRA PARA DEFINIR ORIENTACION", cv::Point(20, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,0,255), 2);

        if (isDragging) {
            // Dibujar línea desde donde se pulsó hasta donde está el ratón
            cv::line(cvImg, cv::Point(dragStartX, dragStartY), cv::Point(dragEndX, dragEndY), cv::Scalar(0, 255, 255), 2);
            cv::circle(cvImg, cv::Point(dragStartX, dragStartY), 3, cv::Scalar(0, 0, 255), -1);
        }
    }
    else {
        // Verificar límites antes de pintar
        if(px >= 0 && px < cvImg.cols && py >= 0 && py < cvImg.rows) {
            // Círculo del robot
            cv::circle(cvImg, cv::Point(px, py), 5, cv::Scalar(0,0,255), -1); // Rojo
            // Línea de dirección
            cv::line(cvImg, cv::Point(px, py), cv::Point(px_a, py_a), cv::Scalar(0,0,255), 2);
        }
    }

    // Mostrar estado
    std::string modeStr = (navEngine.getMode() == MODE_MAPPING) ? "MAPPING" : "LOCALIZACION";
    cv::putText(cvImg, modeStr, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,255,0), 2);


    // 4. Dibujar ruta planificada si existe
    if (planner.getState() == PLAN_NAVIGATING) {
        auto pathPoints = planner.getCurrentPath();

        std::vector<cv::Point> cvPath;
        for (const auto &p : pathPoints) {
            int px = grid->x2idx(p.x);
            int py = grid->y2idx(p.y); // Recuerda verificar si necesitas invertir Y según tu setup
            // Si usaste flip vertical en la imagen:
            // int py_img = cvImg.rows - 1 - py;
            // O usa la misma lógica que usaste para dibujar al robot.
            cvPath.push_back(cv::Point(px, py));
        }

        if (cvPath.size() > 1) {
            cv::polylines(cvImg, cvPath, false, cv::Scalar(0, 255, 0), 2); // Línea VERDE
        }
    }

    cv::imshow("Navegacion", cvImg);

    // Configuración del ratón (solo una vez)
    static bool callbackSet = false;
    if (!callbackSet) {
        cv::setMouseCallback("Navegacion", SpecificWorker::onMouse, nullptr);
        callbackSet = true;
    }
	if (navEngine.getMode() == MODE_LOCALIZATION)
    {
        // 1. Obtener métricas
        auto metrics = navEngine.getMetrics();

		bool quality_bad = (metrics.quality < -200.0); // Ajusta este umbral empíricamente
        bool lost_uncertainty = (metrics.std_x > 0.5);

        cv::Scalar color;
        std::string estado;

        if (quality_bad) {
            color = cv::Scalar(0, 0, 255); // ROJO
            estado = "FUERA DEL MAPA / BLOQUEADO";
        } else if (lost_uncertainty) {
            color = cv::Scalar(0, 255, 255); // AMARILLO
            estado = "DISPERSO";
        } else {
            color = cv::Scalar(0, 255, 0); // VERDE
            estado = "OK";
        }

        // 2. Semáforo de confianza
        cv::Scalar colorConfianza;
        float incertidumbre_media = (metrics.std_x + metrics.std_y) / 2.0;
        if (incertidumbre_media < 0.15) colorConfianza = cv::Scalar(0, 255, 0); // Verde (Bueno)
        else if (incertidumbre_media < 0.5) colorConfianza = cv::Scalar(0, 255, 255); // Amarillo (Regular)
        else colorConfianza = cv::Scalar(0, 0, 255); // Rojo (Malo/Perdido)

        // 3. Dibujar círculo de incertidumbre (Radio = 3 sigmas para 99% confianza)
        // Convertimos metros a píxeles.
        // Nota: grid->getResolution() suele ser 0.05m/pixel
        float resolution = grid->getResolution();
        int radius_px = (incertidumbre_media * 3.0) / resolution;

        if (radius_px < 2) radius_px = 2; // Mínimo visible
		//qInfo() << "Drawing uncertainty circle with radius (px): " << radius_px << "uncertainty (m): " << incertidumbre_media ;
        // Dibujar círculo alrededor del robot (px, py son las coords del robot calculadas antes)
        cv::circle(cvImg, cv::Point(px, py), radius_px, colorConfianza, 1);

		std::string txt = "Q: " + std::to_string((int)metrics.quality) + " | S: " + std::to_string(metrics.std_x).substr(0,4);

        cv::putText(cvImg, txt, cv::Point(10, cvImg.rows - 40), cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
        cv::putText(cvImg, estado, cv::Point(10, cvImg.rows - 10), cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
		// std::cout << "Localization metrics - State: " << estado << ", Std X: " << metrics.std_x << ", Std Y: " << metrics.std_y << std::endl;
    }
    cv::waitKey(1);


}

// Callback de Odometría (Se ejecuta en otro hilo)
void SpecificWorker::FullPoseEstimationPub_newFullPose(RoboCompFullPoseEstimation::FullPoseEuler pose)
{
    if(std::isnan(pose.vx) || std::isnan(pose.vy)) return;

    // Empaquetamos y mandamos al Tracker para que integre
    VelocityData v;
	// v.vx = pose.vy / 1000.0;
 //
 //    // Tu X (Derecha) -> MRPT Y (Izquierda)
 //    // ANTES: v.vy = -pose.vx; (Derecha -> -Izquierda = Derecha)
 //    // AHORA: v.vy = pose.vx;  (Derecha -> Izquierda)
 //    // Al invertir este signo, rompemos el efecto espejo.
 //    v.vy = -pose.vx / 1000.0;
 //    v.w  = -pose.vrz;

    /// EN WEBOTS vienen con X+ hacia delante e Y+ hacia la izquierda, así que no necesitamos invertir el signo de vx ni de vrz. Solo convertir a m/s.

    v.vx = pose.vx;

    // Tu X (Derecha) -> MRPT Y (Izquierda)
    // ANTES: v.vy = -pose.vx; (Derecha -> -Izquierda = Derecha)
    // AHORA: v.vy = pose.vx;  (Derecha -> Izquierda)
    // Al invertir este signo, rompemos el efecto espejo.
    v.vy = pose.vy;
    v.w  = pose.vrz;

    v.timestamp_seconds = static_cast<double>(pose.timestamp) / 1000.0;
    odomTracker.processNewVelocity(v);
}

void SpecificWorker::screenToRawPixels(int sx, int sy, int &rx, int &ry)
{
	// Inversa: (Screen - Offset) / Scale
    rx = (int)((sx - current_offset_x) / current_effective_scale);
    ry = (int)((sy - current_offset_y) / current_effective_scale);
}

std::string SpecificWorker::askUserForMap()
{
    namespace fs = std::filesystem;
    std::string path = "results"; // Carpeta donde guardas los mapas
    std::vector<std::string> maps;

    // Restauramos la terminal para poder usar std::cin cómodamente
    restoreTerminal();

    std::cout << "\n========================================" << std::endl;
    std::cout << "   SELECCION DE MAPA PARA LOCALIZAR" << std::endl;
    std::cout << "========================================" << std::endl;

    int idx = 0;
    if (fs::exists(path)) {
        for (const auto & entry : fs::directory_iterator(path)) {
            if (entry.is_directory()) {
                std::string mapName = entry.path().filename().string();
                // Filtramos carpetas temporales o vacías si quieres
                maps.push_back(entry.path().string());
                std::cout << " [" << idx++ << "] " << mapName << std::endl;
            }
        }
    }

    if (maps.empty()) {
        std::cout << " >> No se encontraron mapas en 'results/'." << std::endl;
        initTerminal(); // Volvemos a modo raw antes de salir
        return "";
    }

    std::cout << "\n > Introduce el numero del mapa (0-" << maps.size()-1 << "): ";
    int selection = -1;
    if (std::cin >> selection) {
        // Limpiamos buffer por si acaso
    } else {
        std::cin.clear(); // Limpiar errores
        std::cin.ignore(10000, '\n');
    }

    initTerminal(); // IMPORTANTE: Volver a activar modo raw para kbhit()

    if (selection >= 0 && selection < (int)maps.size()) {
        return maps[selection];
    }

    std::cout << " >> Seleccion invalida." << std::endl;
    return "";
}

void SpecificWorker::emergency() { std::cout << "Emergency worker" << std::endl; }
void SpecificWorker::restore() { std::cout << "Restore worker" << std::endl; }

int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    return 0;
}

void SpecificWorker::initTerminal()
{
    // Guardamos configuración actual
    tcgetattr(STDIN_FILENO, &oldt);
    struct termios newt = oldt;

    // Desactivamos el modo canónico (esperar enter) y el eco (ver letras)
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    // Hacemos que la lectura sea no bloqueante
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
}

void SpecificWorker::restoreTerminal()
{
    // Restauramos configuración original
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

int SpecificWorker::kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if(ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

char SpecificWorker::getch(void)
{
    char c = 0;
    if (read(STDIN_FILENO, &c, 1) < 0) return 0;
    return c;
}


/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->correctOdometer(int x, int z, float alpha)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->getBasePose(int x, int z, float alpha)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->getBaseState(RoboCompGenericBase::TBaseState state)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->resetOdometer()
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->setOdometer(RoboCompGenericBase::TBaseState state)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->setOdometerPose(int x, int z, float alpha)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->setSpeedBase(float adv, float rot)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->stopBase()

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams
/**************************************/
// From the RoboCompOmniRobot you can call this methods:
// RoboCompOmniRobot::void this->omnirobot_proxy->correctOdometer(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->getBasePose(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->getBaseState(RoboCompGenericBase::TBaseState state)
// RoboCompOmniRobot::void this->omnirobot_proxy->resetOdometer()
// RoboCompOmniRobot::void this->omnirobot_proxy->setOdometer(RoboCompGenericBase::TBaseState state)
// RoboCompOmniRobot::void this->omnirobot_proxy->setOdometerPose(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->setSpeedBase(float advx, float advz, float rot)
// RoboCompOmniRobot::void this->omnirobot_proxy->stopBase()
