# Sources set
LIST(APPEND SOURCES
  ../src/specificworker.cpp
)

# Headers set
LIST(APPEND HEADERS
  ../src/specificworker.h
)

INCLUDE( $ENV{ROBOCOMP}/cmake/modules/opencv4.cmake )

add_compile_options(
    -O3 
    -march=native 
    -flto=auto
    -fno-plt
)
add_definitions(-DNDEBUG -fmax-errors=1)
add_link_options(-flto=auto)

find_package(OpenMP REQUIRED)
LIST(APPEND LIBS ${LIBS} OpenMP::OpenMP_CXX)

# rs_driver (RoboSense SDK) is optional: without it (or with -DLIDAR3D_SIMULATOR_ONLY=ON) the
# component is built in simulator-only mode (simulator must be true in the config).
option(LIDAR3D_SIMULATOR_ONLY "Build without rs_driver (simulation only)" OFF)

if (NOT LIDAR3D_SIMULATOR_ONLY)
    find_package(rs_driver QUIET)
    if (NOT rs_driver_FOUND)
        message(WARNING "rs_driver not found -> building lidar3D in SIMULATOR-ONLY mode")
        set(LIDAR3D_SIMULATOR_ONLY ON)
    endif()
endif()

if (LIDAR3D_SIMULATOR_ONLY)
    message(STATUS "lidar3D: SIMULATOR-ONLY build (no rs_driver)")
    add_definitions(-DLIDAR3D_SIMULATOR_ONLY)
else()
    message(STATUS "lidar3D: rs_driver found -> real + simulated lidar support")
    include_directories(${rs_driver_INCLUDE_DIRS})
    LIST(APPEND LIBS ${LIBS} ${rs_driver_LIBRARIES})
endif()

