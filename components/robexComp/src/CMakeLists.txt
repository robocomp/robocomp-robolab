cmake_minimum_required(VERSION 2.6)
PROJECT( robexComp )

# Sources set
SET ( SOURCES
  robexComp.cpp
  robexhandler.cpp
  monitor.cpp
  commonbehaviorI.cpp
  differentialrobotI.cpp
  joystickadapterI.cpp
  worker.cpp
  $ENV{ROBOCOMP}/classes/q4serialport/q4serialport.cpp
  $ENV{ROBOCOMP}/classes/qlog/qlog.cpp
)

# Headers set
SET ( HEADERS
  commonbehaviorI.h
  handler.h
  robexhandler.h
  monitor.h
  joystickadapterI.h
  worker.h
  differentialrobotI.h
  $ENV{ROBOCOMP}/classes/q4serialport/q4serialport.h
)

# RoboComp
INCLUDE( $ENV{ROBOCOMP}/cmake/robocomp.cmake )
ROBOCOMP_INITIALIZE( $ENV{ROBOCOMP} )
ROBOCOMP_WRAP_ICE( DifferentialRobot Logger CommonBehavior JoystickAdapter)

# Qt4
ADD_DEFINITIONS( -Wall -DQT_DLL -DQT_GUI_LIB -DQT_CORE_LIB )
FIND_PACKAGE( Qt4 REQUIRED )
INCLUDE( ${QT_USE_FILE} )
QT4_WRAP_CPP( MOC_SOURCES ${HEADERS} )

# Specify construction and link process
SET (EXECUTABLE_OUTPUT_PATH ../bin)
ADD_EXECUTABLE( robexComp ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS} )
TARGET_LINK_LIBRARIES( robexComp ${QT_LIBRARIES} ${LIBS})
INSTALL(FILES ${EXECUTABLE_OUTPUT_PATH}/robexComp DESTINATION /opt/robocomp/bin/ )
