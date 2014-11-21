cmake_minimum_required(VERSION 2.6)
PROJECT( joystickOmniComp )

# Sources set
SET ( SOURCES
  joystickComp.cpp
  joystickI.cpp
  joystickhandler.cpp
  $ENV{ROBOCOMP}/classes/qjoystick/qjoystick.cpp
  $ENV{ROBOCOMP}/classes/rapplication/rapplication.cpp
)

# Headers set
SET ( HEADERS
  joystickI.h
  joystickhandler.h
  $ENV{ROBOCOMP}/classes/qjoystick/qjoystick.h
)

# RoboComp
INCLUDE( $ENV{ROBOCOMP}/cmake/robocomp.cmake )
ROBOCOMP_INITIALIZE( $ENV{ROBOCOMP}/ )
ROBOCOMP_WRAP_ICE( JoyStick OmniRobot )

# Qt4
ADD_DEFINITIONS( -Wall -DQT_DLL -DQT_GUI_LIB -DQT_CORE_LIB )
FIND_PACKAGE( Qt4 REQUIRED )
INCLUDE( ${QT_USE_FILE} )
QT4_WRAP_CPP( MOC_SOURCES ${HEADERS} )


# Specify construction and link process
SET (EXECUTABLE_OUTPUT_PATH ../bin)
ADD_EXECUTABLE( joystickOmniComp ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS} )
TARGET_LINK_LIBRARIES( joystickOmniComp ${QT_LIBRARIES} ${LIBS})
INSTALL(FILES ${EXECUTABLE_OUTPUT_PATH}/joystickOmniComp DESTINATION /opt/robocomp/bin/ PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
