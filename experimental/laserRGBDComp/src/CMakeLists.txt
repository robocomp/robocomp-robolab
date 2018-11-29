cmake_minimum_required(VERSION 2.6)
PROJECT( laserrgbdComp )

# Sources set
SET ( SOURCES
  laserrgbComp.cpp
  laserI.cpp
  monitor.cpp
  commonbehaviorI.cpp
  worker.cpp
  $ENV{ROBOCOMP}/classes/extendedRangeSensor/extendedRangeSensor.cpp
  $ENV{ROBOCOMP}/classes/rapplication/rapplication.cpp
  $ENV{ROBOCOMP}/classes/qlog/qlog.cpp
)

# Headers set
SET ( HEADERS
  laserI.h
  monitor.h
  commonbehaviorI.h
  worker.h
)


ADD_DEFINITIONS( -std=c++11 )


# RoboComp
INCLUDE( $ENV{ROBOCOMP}/cmake/robocomp.cmake )
ROBOCOMP_INITIALIZE( $ENV{ROBOCOMP}/ )
ROBOCOMP_WRAP_ICE( CommonBehavior  Laser OmniRobot DifferentialRobot JointMotor RGBD JointMotor RGBDBus )

# PCL
find_package(PCL 1.6 REQUIRED COMPONENTS common io )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} /usr/local/lib )
add_definitions(${PCL_DEFINITIONS})

# IPP
INCLUDE( $ENV{ROBOCOMP}/cmake/modules/ipp.cmake )

# OPENSG
INCLUDE ($ENV{ROBOCOMP}/cmake/modules/opensg.cmake )

# OpenMP
ADD_DEFINITIONS( -fopenmp -g )
SET(LIBS ${LIBS} -lgomp)

# Qt4
ADD_DEFINITIONS( -Wall -DQT_DLL -DQT_GUI_LIB -DQT_CORE_LIB )
FIND_PACKAGE( Qt4 REQUIRED )
SET( QT_USE_QTGUI TRUE )
SET( QT_USE_QTOPENGL TRUE )
SET( QT_USE_QTXML TRUE )
INCLUDE( ${QT_USE_FILE} )
QT4_WRAP_CPP( MOC_SOURCES ${HEADERS} )
QT4_WRAP_UI( UI_HEADERS guiDlg.ui )


# Specify construction and link process
SET (EXECUTABLE_OUTPUT_PATH ../bin)
ADD_EXECUTABLE( laserrgbdComp ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS} )
TARGET_LINK_LIBRARIES( laserrgbdComp ${QT_LIBRARIES} ${LIBS} ${PCL_LIBRARIES}   ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES} -lpcl_search ${PCL_USER_LIBRARIES} glut robocomp_innermodel OpenThreads osgGA osgDB osgUtil osgText ${Ice_LIBRARIES})
# TARGET_LINK_LIBRARIES( rcis         ${QT_LIBRARIES} ${LIBS} glut robocomp_innermodel OpenThreads osgGA osgDB osgUtil osgText librapid.a -L${CMAKE_CURRENT_BINARY_DIR}/lib)

INSTALL(FILES ${EXECUTABLE_OUTPUT_PATH}/laserrgbdComp DESTINATION /opt/robocomp/bin/ PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
