# Zero-copy DDS media plane (shared lib in active_inference/common/media_transport).
# zed_camera is NOT a DSR agent, so FastDDS is linked standalone (see media_bench),
# not via the dsr cmake module. Anchored on this file's dir so it is include-location
# independent.
set(MEDIA_TRANSPORT_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../../../../active_inference/common/media_transport)
set(MEDIA_TRANSPORT_IDL ${MEDIA_TRANSPORT_DIR}/generated/idl)

# Sources set
LIST(APPEND SOURCES
  ../src/specificworker.cpp
  ../src/dds_publisher.cpp
  ${MEDIA_TRANSPORT_DIR}/media_transport.cpp
  ${MEDIA_TRANSPORT_IDL}/image_framePubSubTypes.cxx
  ${MEDIA_TRANSPORT_IDL}/image_frameTypeObjectSupport.cxx
  ${MEDIA_TRANSPORT_IDL}/image360_framePubSubTypes.cxx
  ${MEDIA_TRANSPORT_IDL}/image360_frameTypeObjectSupport.cxx
  ${MEDIA_TRANSPORT_IDL}/lidar_framePubSubTypes.cxx
  ${MEDIA_TRANSPORT_IDL}/lidar_frameTypeObjectSupport.cxx
  ${MEDIA_TRANSPORT_IDL}/imu_framePubSubTypes.cxx
  ${MEDIA_TRANSPORT_IDL}/imu_frameTypeObjectSupport.cxx
)

# Headers set
LIST(APPEND HEADERS
  ../src/specificworker.h
  ../src/dds_publisher.h
)

include_directories(${MEDIA_TRANSPORT_DIR})


INCLUDE( $ENV{ROBOCOMP}/cmake/modules/opencv4.cmake )

if(POLICY CMP0146)
  cmake_policy(SET CMP0146 OLD)
endif()

if(POLICY CMP0153)
  cmake_policy(SET CMP0153 OLD)
endif()

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

# no-char8_t to avoid a problem with ICE Connection lib.
add_definitions(-O3 -march=native -fmax-errors=1 -fno-char8_t)

find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    LIST(APPEND LIBS ${LIBS} OpenMP::OpenMP_CXX)
endif()

option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

if (NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()

find_package(ZED 3 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} REQUIRED)
find_package(OpenMP REQUIRED)


include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

if (LINK_SHARED_ZED)
    SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
else()
    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif()

LIST(APPEND LIBS ${LIBS} ${ZED_LIBS} )

# FastDDS for the standalone media plane (rc::media). Imported targets carry their
# own include dirs, so no extra include_directories() is needed for FastDDS itself.
find_package(fastcdr REQUIRED)
find_package(fastdds REQUIRED)
LIST(APPEND LIBS ${LIBS} fastdds fastcdr)

add_definitions(-O3 -fmax-errors=1 -fno-char8_t)

set(CXX_STANDARD 23)

