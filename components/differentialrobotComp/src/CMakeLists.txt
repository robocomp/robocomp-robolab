cmake_minimum_required(VERSION 2.6)
PROJECT( differentialrobotComp )

# Gazebo
INCLUDE( $ENV{ROBOCOMP}/cmake/modules/gazebo.cmake )
IF (GAZEBO_FOUND EQUAL 0)
  MESSAGE(STATUS "\n\nGazebo handler will NOT be built!\n\n")
ELSE (GAZEBO_FOUND EQUAL 0)
  MESSAGE(STATUS "\n\nGazebo handler WILL be built.\n\n")
ENDIF (GAZEBO_FOUND EQUAL 0)

# Player
INCLUDE( $ENV{ROBOCOMP}/cmake/modules/player.cmake )
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
  genericbaseI.cpp
  worker.cpp
  $ENV{ROBOCOMP}/classes/q4serialport/q4serialport.cpp
  $ENV{ROBOCOMP}/classes/qlog/qlog.cpp
)

# RoboComp
INCLUDE( $ENV{ROBOCOMP}/cmake/robocomp.cmake )
ROBOCOMP_INITIALIZE( $ENV{ROBOCOMP} )
ROBOCOMP_WRAP_ICE( DifferentialRobot Logger CommonBehavior GenericBase)

# Qt4
ADD_DEFINITIONS( -Wall -DQT_DLL -DQT_GUI_LIB -DQT_CORE_LIB )
FIND_PACKAGE( Qt4 REQUIRED )
INCLUDE( ${QT_USE_FILE} )
SET( CMAKE_AUTOMOC TRUE )

# Specify construction and link process
SET (EXECUTABLE_OUTPUT_PATH ../bin)
ADD_EXECUTABLE( differentialrobotComp ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS} )
TARGET_LINK_LIBRARIES( differentialrobotComp ${QT_LIBRARIES} ${LIBS} ${Ice_LIBRARIES})
INSTALL(FILES ${EXECUTABLE_OUTPUT_PATH}/differentialrobotComp DESTINATION /opt/robocomp/bin/ )
