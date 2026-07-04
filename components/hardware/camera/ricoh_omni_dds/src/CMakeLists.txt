# Zero-copy DDS media plane (Image360) — shared lib in active_inference/common/media_transport.
# This component is NOT a DSR agent, so FastDDS is linked standalone (see media_bench),
# not via the dsr cmake module. Anchored on this file's dir (include-location independent).
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

# OpenCV (cv::Mat, VideoCapture/VideoWriter with the GStreamer backend, imgproc, ...)
INCLUDE( $ENV{ROBOCOMP}/cmake/modules/opencv4.cmake )

# FastDDS for the standalone media plane (rc::media).
find_package(fastcdr REQUIRED)
find_package(fastdds REQUIRED)
LIST(APPEND LIBS ${LIBS} fastdds fastcdr)

add_compile_options(
    -O3 
    -fno-plt
)
add_definitions(-DNDEBUG -fmax-errors=1)
add_link_options(-flto=auto)

