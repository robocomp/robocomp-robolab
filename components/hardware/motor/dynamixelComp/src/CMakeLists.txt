cmake_minimum_required(VERSION 2.6)
PROJECT( dynamixelComp )

# Sources set
SET ( SOURCES
  jointmotorComp.cpp
  jointmotorI.cpp
  worker.cpp
  dynamixel_serial.cpp
  dynamixel_sdk.cpp
  monitor.cpp
  servo.cpp
  commonbehaviorI.cpp
  $ENV{ROBOCOMP}/classes/rapplication/rapplication.cpp
  $ENV{ROBOCOMP}/classes/qlog/qlog.cpp
)

# Headers set
SET ( HEADERS
  worker.h
  monitor.h
  commonbehaviorI.h
)

# RoboComp
INCLUDE( $ENV{ROBOCOMP}/cmake/robocomp.cmake )
INCLUDE ($ENV{ROBOCOMP}/cmake/modules/dynamixel.cmake )

ROBOCOMP_INITIALIZE( $ENV{ROBOCOMP}/ )
ROBOCOMP_WRAP_ICE( JointMotor CommonBehavior)

# Qt
INCLUDE( $ENV{ROBOCOMP}/cmake/modules/qt.cmake )


# Specify construction and link process
SET (EXECUTABLE_OUTPUT_PATH ../bin)
ADD_EXECUTABLE( dynamixelComp ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS} )
TARGET_LINK_LIBRARIES( dynamixelComp ${QT_LIBRARIES}   ${LIBS} -lusb ${Ice_LIBRARIES})
INSTALL(FILES ${EXECUTABLE_OUTPUT_PATH}/dynamixelComp DESTINATION /opt/robocomp/bin/ PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
