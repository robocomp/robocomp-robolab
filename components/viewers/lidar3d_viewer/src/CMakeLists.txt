# Sources set
LIST(APPEND SOURCES
  ../src/specificworker.cpp
)
# Headers set
LIST(APPEND HEADERS
  ../src/specificworker.h
)

add_definitions(-O3  -fmax-errors=1 -fno-char8_t -I/usr/include/x86_64-linux-gnu/qt6/QtOpenGLWidgets)
INCLUDE( $ENV{ROBOCOMP}/cmake/modules/opencv4.cmake )
SET (LIBS ${LIBS} QGLViewer-qt6 Qt6OpenGLWidgets)
