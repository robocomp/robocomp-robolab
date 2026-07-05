# Zero-copy DDS media plane (LidarFrame) — shared lib in active_inference/common/media_transport.
# This component is NOT a DSR agent, so FastDDS is linked standalone, not via the dsr cmake module.
set(MEDIA_TRANSPORT_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../../../../active_inference/common/media_transport)
set(MEDIA_TRANSPORT_IDL ${MEDIA_TRANSPORT_DIR}/generated/idl)

# Sources set
LIST(APPEND SOURCES
  ../src/specificworker.cpp
  ../src/dds_publisher.cpp
  ../src/mesh_filter.cpp
  ../src/urdf_mesh_loader.cpp
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
  ../src/mesh_filter.h
  ../src/urdf_mesh_loader.h
)

# Robot-body self-filter deps (Embree ray query, assimp mesh load, tinyxml2 URDF parse).
LIST(APPEND LIBS ${LIBS} embree4 assimp tinyxml2)

include_directories(${MEDIA_TRANSPORT_DIR})

INCLUDE( $ENV{ROBOCOMP}/cmake/modules/opencv4.cmake )

# FastDDS for the standalone media plane (rc::media).
find_package(fastcdr REQUIRED)
find_package(fastdds REQUIRED)
LIST(APPEND LIBS ${LIBS} fastdds fastcdr)

add_compile_options(
    -O3 
    -march=native 
    -flto=auto
    -fno-plt
)
add_definitions(-DNDEBUG -fmax-errors=1)
add_link_options(-flto=auto)

find_package(rs_driver REQUIRED)
find_package(OpenMP REQUIRED)
include_directories(${rs_driver_INCLUDE_DIRS})

LIST(APPEND LIBS ${LIBS} ${rs_driver_LIBRARIES}  OpenMP::OpenMP_CXX)

