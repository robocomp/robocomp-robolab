# Sources set
LIST(APPEND SOURCES
  ../src/specificworker.cpp
)

# Headers set
LIST(APPEND HEADERS
  ../src/specificworker.h
)

INCLUDE( $ENV{ROBOCOMP}/cmake/modules/opencv4.cmake )

add_definitions(-O3 -Wall -fmax-errors=1 -fno-char8_t)

find_package(rs_driver REQUIRED)
include_directories(${rs_driver_INCLUDE_DIRS})

LIST(APPEND LIBS ${LIBS} ${rs_driver_LIBRARIES} Open3D gomp)

