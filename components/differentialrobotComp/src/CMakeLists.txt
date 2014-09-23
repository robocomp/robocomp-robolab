cmake_minimum_required(VERSION 2.6)
PROJECT( differentialrobotComp )

# Gazebo
INCLUDE( $ENV{ROBOCOMP}/CMake/gazebo.cmake )
IF (GAZEBO_FOUND EQUAL 0)
  MESSAGE(STATUS "\n\nGazebo handler will NOT be built!\n\n")
ELSE (GAZEBO_FOUND EQUAL 0)
  MESSAGE(STATUS "\n\nGazebo handler WILL be built.\n\n")
ENDIF (GAZEBO_FOUND EQUAL 0)

# Player
INCLUDE( $ENV{ROBOCOMP}/CMake/player.cmake )
IF (PLAYER_FOUND EQUAL 0)
  MESSAGE(STATUS "\n\nPlayer handler will NOT be built!\n\n")
ELSE (PLAYER_FOUND EQUAL 0)
  MESSAGE(STATUS "\n\nPlayer handler WILL be built.\n\n")
ENDIF (PLAYER_FOUND EQUAL 0)

# Sources set
SET ( SOURCES
  differentialrobotComp.cpp
  gazebohandler.cpp
  playerhandler.cpp
  monitor.cpp
  commonbehaviorI.cpp
  differentialrobotI.cpp
  worker.cpp
  $ENV{ROBOCOMP}/Classes/q4serialport/q4serialport.cpp
  $ENV{ROBOCOMP}/Classes/qlog/qlog.cpp
)

# Headers set
SET ( HEADERS
  commonbehaviorI.h
  handler.h
  gazebohandler.h
  playerhandler.h
  monitor.h
  worker.h
  differentialrobotI.h
  $ENV{ROBOCOMP}/Classes/q4serialport/q4serialport.h
)

# RoboComp
INCLUDE( $ENV{ROBOCOMP}/CMake/robocomp.cmake )
ROBOCOMP_INITIALIZE( $ENV{ROBOCOMP} )
ROBOCOMP_WRAP_ICE( DifferentialRobot Logger CommonBehavior)

# Qt4
ADD_DEFINITIONS( -Wall -DQT_DLL -DQT_GUI_LIB -DQT_CORE_LIB )
FIND_PACKAGE( Qt4 REQUIRED )
INCLUDE( ${QT_USE_FILE} )
QT4_WRAP_CPP( MOC_SOURCES ${HEADERS} )

# Specify construction and link process
SET (EXECUTABLE_OUTPUT_PATH ../bin)
ADD_EXECUTABLE( differentialrobotComp ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS} )
TARGET_LINK_LIBRARIES( differentialrobotComp ${QT_LIBRARIES} ${LIBS})
INSTALL(FILES ${EXECUTABLE_OUTPUT_PATH}/differentialrobotComp DESTINATION /opt/robocomp/bin/ )
