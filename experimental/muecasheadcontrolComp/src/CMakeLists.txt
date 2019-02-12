cmake_minimum_required(VERSION 2.6)
PROJECT( muecasheadcontrolcomp )

INCLUDE ( CMakeListsSpecific.txt)

# Sources set
SET ( SOURCES
  ${SOURCES}
  muecasheadcontrolcomp.cpp
  commonheadI.cpp
  muecasheadcontrolI.cpp
  joystickadapterI.cpp
  $ENV{ROBOCOMP}/classes/rcdraw/rcdraw.cpp
  genericmonitor.cpp
  commonbehaviorI.cpp
  genericworker.cpp
  $ENV{ROBOCOMP}/classes/rapplication/rapplication.cpp
  $ENV{ROBOCOMP}/classes/qlog/qlog.cpp
)

# Headers set
SET ( HEADERS
  ${HEADERS}
  commonheadI.h
  muecasheadcontrolI.h
  joystickadapterI.h
  $ENV{ROBOCOMP}/classes/rcdraw/rcdraw.h
  genericmonitor.h
  commonbehaviorI.h
  genericworker.h
)
#ROBOCOMP
INCLUDE( $ENV{ROBOCOMP}/cmake/robocomp.cmake )
ROBOCOMP_INITIALIZE( $ENV{ROBOCOMP}/ )
ROBOCOMP_WRAP_ICE( CommonBehavior  muecasheadcontrol JoystickAdapter Camera CommonHead  JointMotor DifferentialRobot IMU)
SET (EXECUTABLE_OUTPUT_PATH ../bin)

# ipp
INCLUDE ($ENV{ROBOCOMP}/cmake/modules/ipp.cmake )
# robocomp
INCLUDE ($ENV{ROBOCOMP}/cmake/robocomp.cmake )
# qmat
INCLUDE ($ENV{ROBOCOMP}/cmake/modules/qmat.cmake )
# opencv
INCLUDE ($ENV{ROBOCOMP}/cmake/modules/opencv.cmake )
# Qt4
ADD_DEFINITIONS( -Wall -DQT_DLL -DQT_GUI_LIB -DQT_CORE_LIB )
FIND_PACKAGE( Qt4 REQUIRED )
SET( QT_USE_QTGUI TRUE )
SET( QT_USE_QTOPENGL TRUE )
SET( QT_USE_QTXML TRUE )
SET( QT_USE_QTSTATE TRUE )
SET( QT_USE_QTSTATEMACHINE TRUE )
INCLUDE( ${QT_USE_FILE} )
QT4_WRAP_CPP( MOC_SOURCES ${HEADERS} )
QT4_WRAP_UI( UI_HEADERS guiDlg.ui )
# Specify construction and link process
ADD_EXECUTABLE( muecasheadcontrolcomp ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS} )
TARGET_LINK_LIBRARIES( muecasheadcontrolcomp ${QT_LIBRARIES} ${LIBS} ${STATIC_LIBS} ${Ice_LIBRARIES})
INSTALL(FILES ${EXECUTABLE_OUTPUT_PATH}/muecasheadcontrolcomp DESTINATION /opt/robocomp/bin/ PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
