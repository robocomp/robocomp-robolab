cmake_minimum_required(VERSION 2.6)
PROJECT( faulhabercompComp )

INCLUDE ( CMakeListsSpecific.txt)

# Sources set
SET ( SOURCES
  ${SOURCES}
  faulhabercompComp.cpp
  jointmotorI.cpp
  genericmonitor.cpp
  commonbehaviorI.cpp
  genericworker.cpp
  $ENV{ROBOCOMP}/classes/rapplication/rapplication.cpp
  $ENV{ROBOCOMP}/classes/qlog/qlog.cpp
)

# Headers set
SET ( HEADERS
  ${HEADERS}
  jointmotorI.h
  genericmonitor.h
  commonbehaviorI.h
  genericworker.h

)

# RoboComp
INCLUDE( $ENV{ROBOCOMP}/cmake/robocomp.cmake )
ROBOCOMP_INITIALIZE( $ENV{ROBOCOMP}/ )
ROBOCOMP_WRAP_ICE( CommonBehavior   JointMotor)

SET (EXECUTABLE_OUTPUT_PATH ../bin)

# IPP
# INCLUDE( $ENV{ROBOCOMP}/CMake/ipp.cmake )
# FrameWave
# INCLUDE( $ENV{ROBOCOMP}/CMake/framewave.cmake )

# Qt4
ADD_DEFINITIONS( -Wall -DQT_DLL -DQT_GUI_LIB -DQT_CORE_LIB )
FIND_PACKAGE( Qt4 REQUIRED )
SET( QT_USE_QTGUI TRUE )
SET( QT_USE_QTOPENGL TRUE )
SET( QT_USE_QTXML TRUE )
INCLUDE( ${QT_USE_FILE} )
QT4_WRAP_CPP( MOC_SOURCES ${HEADERS} )

# Specify construction and link process
ADD_EXECUTABLE( faulhaberComp ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS} )
TARGET_LINK_LIBRARIES( faulhaberComp ${QT_LIBRARIES} ${LIBS})

INSTALL(FILES faulhaberComp DESTINATION /opt/robocomp/bin/ PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
