cmake_minimum_required(VERSION 2.6)
PROJECT( primeSenseComp )

# Sources set
SET ( SOURCES
  primesenseComp.cpp
  rgbdI.cpp
  monitor.cpp
  commonbehaviorI.cpp
  worker.cpp
  $ENV{ROBOCOMP}/classes/rapplication/rapplication.cpp
  $ENV{ROBOCOMP}/classes/qlog/qlog.cpp
)

# Headers set
SET ( HEADERS
  rgbdI.h
  monitor.h
  commonbehaviorI.h
  worker.h
)

# RoboComp
INCLUDE( $ENV{ROBOCOMP}/cmake/robocomp.cmake )
ROBOCOMP_INITIALIZE( $ENV{ROBOCOMP}/ )
ROBOCOMP_WRAP_ICE( CommonBehavior RGBD CommonHead JointMotor DifferentialRobot )

# IPP
INCLUDE( $ENV{ROBOCOMP}/cmake/modules/ipp.cmake )
# FrameWave
# INCLUDE( $ENV{ROBOCOMP}/cmake/modules/framewave.cmake )

# Qt4
ADD_DEFINITIONS( -Wall -O3 -DQT_DLL -DQT_GUI_LIB -DQT_CORE_LIB -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED )
FIND_PACKAGE( Qt4 REQUIRED )
SET( QT_USE_QTGUI TRUE )
SET( QT_USE_QTOPENGL TRUE )
SET( QT_USE_QTXML TRUE )
INCLUDE( ${QT_USE_FILE} )
QT4_WRAP_CPP( MOC_SOURCES ${HEADERS} )


# OpenNI
ADD_DEFINITIONS( -I/usr/include/ni )


# Specify construction and link process
SET (EXECUTABLE_OUTPUT_PATH ../bin)
ADD_EXECUTABLE( primeSenseComp ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS} )
TARGET_LINK_LIBRARIES( primeSenseComp ${QT_LIBRARIES} ${LIBS} OpenNI  ${Ice_LIBRARIES})
INSTALL(FILES ${EXECUTABLE_OUTPUT_PATH}/primeSenseComp DESTINATION /opt/robocomp/bin/ PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
