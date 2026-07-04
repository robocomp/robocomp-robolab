cmake_minimum_required(VERSION 3.10)

#Select the compiler standard
if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 11)
    message(FATAL_ERROR "Version of g++ is lower than 11, g++ 11 or higher is required.")
elseif(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 12)
    message(STATUS "Using g++ 11, setting C++20 standard.")
    set(CMAKE_CXX_STANDARD 20)
elseif(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 13)
    message(STATUS "Using g++ 12, setting C++23 standard.")
    set(CMAKE_CXX_STANDARD 23)
elseif(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 15)
    message(STATUS "Using g++ 13 or higher, setting the C++23 standard.")
    set(CMAKE_CXX_STANDARD 23)
else()
    message(STATUS "Using g++ 15 or higher, setting the C++26 standard.")
    set(CMAKE_CXX_STANDARD 26)
endif()


IF(NOT EXISTS $ENV{ROBOCOMP}/cmake)
  MESSAGE(FATAL_ERROR "Couldn't find RoboComp in $ENV{ROBOCOMP}! Exiting...")
ENDIF(NOT EXISTS $ENV{ROBOCOMP}/cmake)

INCLUDE($ENV{ROBOCOMP}/cmake/robocomp.cmake)
INCLUDE($ENV{ROBOCOMP}/cmake/modules/qt.cmake)
INCLUDE (../src/CMakeLists.txt)

ADD_DEFINITIONS ("-DICE_CPP11_MAPPING")
FIND_PACKAGE( Ice REQUIRED COMPONENTS Ice++11 IceStorm++11)

# Sources set
SET (SOURCES
  ${SOURCES}
  $ENV{ROBOCOMP}/classes/rapplication/rapplication.cpp
  $ENV{ROBOCOMP}/classes/sigwatch/sigwatch.cpp
  $ENV{ROBOCOMP}/classes/grafcetStep/GRAFCETStep.cpp
  $ENV{ROBOCOMP}/classes/ConfigLoader/ConfigLoader.cpp
  main.cpp
  genericworker.cpp
  camerargbdsimpleI.cpp
  lidar3dI.cpp
  mediaplaneddsI.cpp
)


#ROBOCOMP
ROBOCOMP_INITIALIZE( $ENV{ROBOCOMP}/ )
ROBOCOMP_IDSL_TO_ICE(IMUPub CameraRGBDSimplePub FullPoseEstimationPub CameraRGBDSimple FullPoseEstimation MediaPlaneDDS Lidar3D IMU)
ROBOCOMP_ICE_TO_SRC(IMUPub CameraRGBDSimplePub FullPoseEstimationPub CameraRGBDSimple FullPoseEstimation MediaPlaneDDS Lidar3D IMU)
SET (EXECUTABLE_OUTPUT_PATH ${RC_COMPONENT_DEVEL_PATH}/bin)

# Specify construction and link process
ADD_EXECUTABLE( zed_camera ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS} )
TARGET_LINK_LIBRARIES( zed_camera ${LIBS}  ${STATIC_LIBS} ${SPECIFIC_LIBS} ${QT_LIBRARIES} ${Ice_LIBRARIES} )
INSTALL(FILES ${EXECUTABLE_OUTPUT_PATH}/zed_camera DESTINATION ${RC_COMPONENT_INSTALL_PATH}/bin/ PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
