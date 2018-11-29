cmake_minimum_required(VERSION 2.6)
PROJECT( cameraComp )

# RoboComp
INCLUDE( $ENV{ROBOCOMP}/cmake/robocomp.cmake )
ROBOCOMP_INITIALIZE( $ENV{ROBOCOMP} )
ROBOCOMP_WRAP_ICE( DifferentialRobot JointMotor Camera CommonHead )

# Gazebo
INCLUDE( $ENV{ROBOCOMP}/cmake/modules/gazebo.cmake )
IF (GAZEBO_FOUND EQUAL 0)
  MESSAGE(STATUS "\n\nGazebo camera capture will NOT be built!\n\n")
ELSE (GAZEBO_FOUND EQUAL 0)
  MESSAGE(STATUS "\n\nGazebo camera capture WILL be built.\n\n")
ENDIF (GAZEBO_FOUND EQUAL 0)

# Prosilica
INCLUDE( $ENV{ROBOCOMP}/cmake/modules/prosilica.cmake )
IF (PROSILICA_FOUND EQUAL 0)
  MESSAGE(STATUS "\n\nProsilica camera capture will NOT be built!\n\n")
ELSE (PROSILICA_FOUND EQUAL 0)
  MESSAGE(STATUS "\n\nProsilica camera capture WILL be built.\n\n")
ENDIF (PROSILICA_FOUND EQUAL 0)

# Flycapture
INCLUDE( $ENV{ROBOCOMP}/cmake/modules/flycapture.cmake )
IF (FLYCAPTURE_FOUND EQUAL 0)
  MESSAGE(STATUS "\n\nFlycapture camera capture will NOT be built!\n\n")
ELSE (PROSILICA_FOUND EQUAL 0)
  MESSAGE(STATUS "\n\nFlycapture camera capture WILL be built.\n\n")
ENDIF (FLYCAPTURE_FOUND EQUAL 0)

# Sources set
SET ( SOURCES ${SOURCES}
  cameraComp.cpp
  cameraI.cpp
  dummycapture.cpp
  ieee1394capture.cpp
  v4l2capture.cpp
  gazebocapture.cpp
  prosilicacapture.cpp
  flycapture.cpp
  $ENV{ROBOCOMP}/classes/rapplication/rapplication.cpp
  $ENV{ROBOCOMP}/classes/logpolar/lpolar.cpp
)

# Headers set
SET ( HEADERS
  cameraI.h
  capturador.h
  dummycapture.h
  ieee1394capture.h
  v4l2capture.h
)

# Qt4
ADD_DEFINITIONS( -g -Wall -DQT_DLL -DQT_GUI_LIB -DQT_CORE_LIB )
FIND_PACKAGE( Qt4 REQUIRED )
INCLUDE( ${QT_USE_FILE} )
QT4_WRAP_CPP( MOC_SOURCES ${HEADERS} )

# IPP
INCLUDE( $ENV{ROBOCOMP}/cmake/modules/ipp.cmake )
IF (IPP_FOUND EQUAL 0)
  MESSAGE(STATUS "\n\nUsing FrameWave for image processing.\n\n")
  INCLUDE( $ENV{ROBOCOMP}/cmake/modules/framewave.cmake )
ELSE (IPP_FOUND EQUAL 0)
  MESSAGE(STATUS "\n\nIPP found.\n\n")
  SET (USE_IPP True)
ENDIF (IPP_FOUND EQUAL 0)

# Specify construction and link process
SET (EXECUTABLE_OUTPUT_PATH ../bin)
ADD_EXECUTABLE( cameraComp ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS} )
TARGET_LINK_LIBRARIES( cameraComp ${QT_LIBRARIES} ${LIBS} -ldc1394 ${Ice_LIBRARIES} )
MESSAGE(STATUS ${LIBS})
INSTALL(FILES ${EXECUTABLE_OUTPUT_PATH}/cameraComp DESTINATION /opt/robocomp/bin/ PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
