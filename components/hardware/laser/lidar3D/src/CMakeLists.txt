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

find_package(rs_driver REQUIRED)
find_package(OpenMP REQUIRED)
include_directories(${rs_driver_INCLUDE_DIRS})

LIST(APPEND LIBS ${LIBS} ${rs_driver_LIBRARIES}  OpenMP::OpenMP_CXX)

