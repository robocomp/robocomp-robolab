cmake_minimum_required(VERSION 2.6)
PROJECT( dunkermotorenComp )

# Sources set
SET ( SOURCES
  dunkermotorenComp.cpp
  jointmotorI.cpp
  worker.cpp
  dunkermotoren.cpp
  dunkermotoren_api.cpp
  monitor.cpp
  servo.cpp
  $ENV{ROBOCOMP}/Classes/qlog/qlog.cpp
  $ENV{ROBOCOMP}/Classes/rapplication/rapplication.cpp
  $ENV{ROBOCOMP}/Classes/q4serialport/q4serialport.cpp
)

# Headers set
SET ( HEADERS
  worker.h
  monitor.h
  $ENV{ROBOCOMP}/Classes/q4serialport/q4serialport.h
)

# RoboComp
INCLUDE( $ENV{ROBOCOMP}/CMake/robocomp.cmake )
ROBOCOMP_INITIALIZE( $ENV{ROBOCOMP}/ )
ROBOCOMP_WRAP_ICE( JointMotor CommonBehavior )

#~ # DUNKERMOTOREN
INCLUDE( $ENV{ROBOCOMP}/CMake/dunker.cmake )
IF (DUNKER_FOUND EQUAL 0)
  MESSAGE(STATUS "\n\nDunker motor handle will NOT be built!\n\n")
ELSE (DUNKER_FOUND EQUAL 0)
  MESSAGE(STATUS "\n\nDunker motor handle WILL be built!\n\n")
ENDIF (DUNKER_FOUND EQUAL 0)

# Qt4
ADD_DEFINITIONS( -Wall -DLINUX -D__LINUX__ -DUNIX -DQT_DLL -DQT_GUI_LIB -DQT_CORE_LIB )
FIND_PACKAGE( Qt4 REQUIRED )
SET( QT_USE_QTGUI TRUE )
SET( QT_USE_QTOPENGL TRUE )
INCLUDE( ${QT_USE_FILE} )
QT4_WRAP_CPP( MOC_SOURCES ${HEADERS} )

# Specify construction and link process
SET (EXECUTABLE_OUTPUT_PATH ../bin)
ADD_EXECUTABLE( dunkermotorenComp ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS} )
TARGET_LINK_LIBRARIES( dunkermotorenComp ${QT_LIBRARIES} ${LIBS})
INSTALL(FILES ${EXECUTABLE_OUTPUT_PATH}/dunkermotorenComp DESTINATION /opt/robocomp/bin/ PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
