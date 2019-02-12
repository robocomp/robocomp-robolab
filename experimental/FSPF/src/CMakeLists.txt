
cmake_minimum_required(VERSION 2.6)
PROJECT(  FSPF )
IF(NOT EXISTS $ENV{ROBOCOMP}/cmake)
MESSAGE(FATAL_ERROR "
 ROBOCOMP variable not set, check your bashrc profile!
")
ENDIF(NOT EXISTS $ENV{ROBOCOMP}/cmake)
INCLUDE( $ENV{ROBOCOMP}/cmake/robocomp.cmake )
INCLUDE ( CMakeListsSpecific.txt)

# Sources set
SET ( SOURCES
  ${SOURCES}
  $ENV{ROBOCOMP}/classes/rapplication/rapplication.cpp
  $ENV{ROBOCOMP}/classes/qlog/qlog.cpp
  main.cpp
  genericmonitor.cpp
  specificmonitor.cpp
  commonbehaviorI.cpp
  genericworker.cpp
  specificworker.cpp
)

# Headers set
SET ( HEADERS
  ${HEADERS}
  genericmonitor.h
  specificmonitor.h
  commonbehaviorI.h
  genericworker.h
  specificworker.h
)
#ROBOCOMP
ROBOCOMP_INITIALIZE( $ENV{ROBOCOMP}/ )
ROBOCOMP_WRAP_ICE( CommonBehavior RGBD JointMotor DifferentialRobot )
SET (EXECUTABLE_OUTPUT_PATH ../bin)

# Qt4
ADD_DEFINITIONS( -Wall -DQT_DLL -DQT_GUI_LIB -DQT_CORE_LIB )
FIND_PACKAGE( Qt4 REQUIRED )
SET( QT_USE_QTGUI TRUE )
SET( QT_USE_QTOPENGL TRUE )
SET( QT_USE_QTXML TRUE )
SET( QT_USE_QTSTATE TRUE )
SET( QT_USE_QTSTATEMACHINE TRUE )
INCLUDE( ${QT_USE_FILE} )

QT4_WRAP_UI( UI_HEADERS mainUI.ui )
QT4_WRAP_CPP( MOC_SOURCES ${HEADERS} )

# Specify construction and link process
ADD_EXECUTABLE( FSPF ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS} )
TARGET_LINK_LIBRARIES( FSPF ${QT_LIBRARIES} ${LIBS} ${STATIC_LIBS} ${SPECIFIC_LIBS} ${Ice_LIBRARIES})
INSTALL(FILES ${EXECUTABLE_OUTPUT_PATH}/FSPF DESTINATION /opt/robocomp/bin/ PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
