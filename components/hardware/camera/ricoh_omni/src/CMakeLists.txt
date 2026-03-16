# Sources set
LIST(APPEND SOURCES
  ../src/specificworker.cpp
)
# Headers set
LIST(APPEND HEADERS
  ../src/specificworker.h
)
# Sources set
# Headers set
INCLUDE( $ENV{ROBOCOMP}/cmake/modules/opencv4.cmake )
set(CMAKE_CXX_STANDARD 20)
add_definitions(-O3 -march=native  -fmax-errors=5 )
SET (LIBS ${LIBS}  )