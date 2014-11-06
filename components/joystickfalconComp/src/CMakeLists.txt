cmake_minimum_required(VERSION 2.6)
PROJECT( joystickfalconComp )

# Sources set
SET ( SOURCES
  joystickfalconComp.cpp
  joystickfalconI.cpp

  falconhandler.cpp
  monitor.cpp
  commonbehaviorI.cpp
  worker.cpp
  $ENV{ROBOCOMP}/classes/rapplication/rapplication.cpp
  $ENV{ROBOCOMP}/classes/qlog/qlog.cpp
)

# Headers set
SET ( HEADERS
  joystickfalconI.h
  falconhandler.h
  monitor.h
  commonbehaviorI.h
  worker.h
)

# RoboComp
INCLUDE( $ENV{ROBOCOMP}/cmake/robocomp.cmake )
ROBOCOMP_INITIALIZE( $ENV{ROBOCOMP}/ )
ROBOCOMP_WRAP_ICE( CommonBehavior Joystickfalcon JoystickAdapter )

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

# Novint Falcon
SET( FALCON_LIBRARIES "/usr/local/lib/libnifalcon.a" usb-1.0 pthread rt )
#SET( FALCON LIBRARIES libinifalcon usb-1.0 pthread rt )

# Specify construction and link process
SET (EXECUTABLE_OUTPUT_PATH ../bin)
ADD_EXECUTABLE( joystickfalconComp ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS} )
TARGET_LINK_LIBRARIES( joystickfalconComp ${QT_LIBRARIES} ${LIBS} ${FALCON_LIBRARIES} )
INSTALL(FILES ${EXECUTABLE_OUTPUT_PATH}/joystickfalconComp DESTINATION /opt/robocomp/bin/ PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
