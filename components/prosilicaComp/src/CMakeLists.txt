cmake_minimum_required(VERSION 2.6)
PROJECT( prosilicaComp )

# Sources set
SET ( SOURCES
  prosilicaComp.cpp
  camerabusI.cpp

  monitor.cpp
  commonbehaviorI.cpp
  worker.cpp
  $ENV{ROBOCOMP}/classes/rapplication/rapplication.cpp
  $ENV{ROBOCOMP}/classes/qlog/qlog.cpp
)

# Headers set
SET ( HEADERS
  camerabusI.h

  monitor.h
  commonbehaviorI.h
  worker.h
)

INCLUDE( $ENV{ROBOCOMP}/cmake/modules/prosilica.cmake )
IF (PROSILICA_FOUND EQUAL 0)
  MESSAGE(FATAL_ERROR "\n\nProsilica's AVT GigE SDK not found.\n\n")
ENDIF (PROSILICA_FOUND EQUAL 0)

# RoboComp
INCLUDE( $ENV{ROBOCOMP}/cmake/robocomp.cmake )
ROBOCOMP_INITIALIZE( $ENV{ROBOCOMP}/ )
ROBOCOMP_WRAP_ICE( CommonBehavior  CameraBus JointMotor DifferentialRobot  DifferentialRobot  JointMotor )

INCLUDE( $ENV{ROBOCOMP}/cmake/modules/ipp.cmake )
IF (IPP_FOUND EQUAL 0)
  MESSAGE(STATUS "\n\nUsing FrameWave for image processing.\n\n")
  INCLUDE( $ENV{ROBOCOMP}/CMake/framewave.cmake )
ELSE (IPP_FOUND EQUAL 0)
  MESSAGE(STATUS "\n\nIPP found.\n\n")
  SET (USE_IPP True)
ENDIF (IPP_FOUND EQUAL 0)

# Qt4
ADD_DEFINITIONS( -Wall -DQT_DLL -DQT_GUI_LIB -DQT_CORE_LIB )
FIND_PACKAGE( Qt4 REQUIRED )
SET( QT_USE_QTGUI TRUE )
SET( QT_USE_QTOPENGL TRUE )
SET( QT_USE_QTXML TRUE )
INCLUDE( ${QT_USE_FILE} )
QT4_WRAP_CPP( MOC_SOURCES ${HEADERS} )


# Specify construction and link process
SET (EXECUTABLE_OUTPUT_PATH ../bin)
ADD_EXECUTABLE( prosilicaComp ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS} )
TARGET_LINK_LIBRARIES( prosilicaComp ${QT_LIBRARIES} ${LIBS} ${Ice_LIBRARIES})
INSTALL(FILES ${EXECUTABLE_OUTPUT_PATH}/prosilicaComp DESTINATION /opt/robocomp/bin/ PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
