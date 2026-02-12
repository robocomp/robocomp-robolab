#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include "odom_tracker.h"
#include "navigation_engine.h"
#include "planning_engine.h"
#include <opencv2/opencv.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

class SpecificWorker : public GenericWorker
{
	Q_OBJECT
public:
	SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);
	~SpecificWorker();

	// Callback Odometría
	void FullPoseEstimationPub_newFullPose(RoboCompFullPoseEstimation::FullPoseEuler pose);

	// Callback estático de OpenCV
	static void onMouse(int event, int x, int y, int flags, void* userdata);

	// Función interna para procesar el clic
	void handleMouse(int event, int x, int y, int flags);

public slots:

	/**
	 * \brief Initializes the worker one time.
	 */
	void initialize();

	/**
	 * \brief Main compute loop of the worker.
	 */
	void compute();

	/**
	 * \brief Handles the emergency state loop.
	 */
	void emergency();

	/**
	 * \brief Restores the component from an emergency state.
	 */
	void restore();

	/**
	 * \brief Performs startup checks for the component.
	 * \return An integer representing the result of the checks.
	 */
	int startup_check();

	void modify_node_slot(std::uint64_t, const std::string &type){};
	void modify_node_attrs_slot(std::uint64_t id, const std::vector<std::string>& att_names){};
	void modify_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type){};
	void modify_edge_attrs_slot(std::uint64_t from, std::uint64_t to, const std::string &type, const std::vector<std::string>& att_names){};
	void del_edge_slot(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){};
	void del_node_slot(std::uint64_t from){};

private:


	bool isDragging = false;
	int dragStartX = 0, dragStartY = 0; // Donde pulsaste
	int dragEndX = 0, dragEndY = 0;     // Donde está el ratón ahora

// Variables de Visualización
    float user_zoom_factor = 1.0f; // 1.0 = Tamaño "Fit to Window". >1 es Zoom In.
    float user_pan_x = 0.0f;       // Desplazamiento manual extra
    float user_pan_y = 0.0f;

    // Variables calculadas en cada frame (para usarlas en el ratón)
    float current_effective_scale = 1.0f;
    float current_offset_x = 0.0f;
    float current_offset_y = 0.0f;

	// Actualiza la firma de tu función auxiliar
    void screenToRawPixels(int sx, int sy, int &rx, int &ry);

	// --- NUEVO: FUNCIONES DE TECLADO TERMINAL ---
	int kbhit(void);  // Detecta si se ha pulsado una tecla
	char getch(void); // Lee la tecla pulsada

	// Configuración original de la terminal para restaurarla al salir
	struct termios oldt;
	void initTerminal();
	void restoreTerminal();

	// Módulos
	OdomTracker odomTracker;
	NavigationEngine navEngine;
	PlanningEngine planner;

	// Visualización
	void updateVis();
	bool startup_check_flag;

	// Flag: ¿Estamos esperando que el usuario haga clic en el mapa?
	bool waitingForInitialPose = false;
	bool waitingForTarget = false;

	// Helper para pedir el mapa por terminal
	std::string askUserForMap();

	float view_scale = 1.0f; // Zoom inicial
    float view_pan_x = 1.0f;
    float view_pan_y = 1.0f;

// Variables para gestionar el arrastre de Pan (Clic Derecho)
    bool isPanning = false;
    int panStartX = 0, panStartY = 0;
};

#endif