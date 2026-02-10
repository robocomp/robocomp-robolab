# Sources set
LIST(APPEND SOURCES
        ../src/specificworker.cpp
        ../src/odom_tracker.cpp
        ../src/navigation_engine.cpp
        ../src/planning_engine.cpp
)

# Headers set
LIST(APPEND HEADERS
        ../src/specificworker.h
        ../src/odom_tracker.h
        ../src/navigation_engine.h
        ../src/planning_engine.cpp
)

# --- MRPT INTEGRATION ---
# Buscamos los paquetes necesarios de MRPT (Ajustar versión 2.0 o superior)
find_package(MRPT REQUIRED slam obs maps system gui nav)

INCLUDE( $ENV{ROBOCOMP}/cmake/modules/opencv4.cmake )

# Añadimos las librerías de MRPT a la lista de enlace
LIST(APPEND LIBS ${MRPT_LIBRARIES})
# ------------------------

LIST(APPEND LIBS ${LIBS})

# no-char8_t to avoid a problem with ICE Connection lib.
add_definitions(-g -fmax-errors=1 -fno-char8_t)