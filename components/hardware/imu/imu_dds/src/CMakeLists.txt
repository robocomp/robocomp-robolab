# Zero-copy DDS media plane (ImuFrame) — shared lib in active_inference/common/media_transport.
# This component is NOT a DSR agent, so FastDDS is linked standalone, not via the dsr cmake module.
set(MEDIA_TRANSPORT_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../../../../active_inference/common/media_transport)
set(MEDIA_TRANSPORT_IDL ${MEDIA_TRANSPORT_DIR}/generated/idl)

# Sources set
# The whole IDL set is unconditional even though this component only writes ImuFrame:
# media_transport.cpp's WriterCore::init references every FrameKind's PubSubType, so a
# single-payload producer still needs them all to link. Same list as lidar3d_dds.
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
  ../src/imu_sample.h
)

# ── Optional: real Phidget Spatial support (IMU.Source = "phidget") ──────────
# OPTIONAL on purpose. The component's other source is the ICE IMU interface, which needs
# nothing extra, so a machine without the Phidget SDK (every simulation box) must still
# build. When the header+lib are found we compile phidget_imu.cpp and define
# HAVE_PHIDGET22; when they are not, IMU.Source = "phidget" fails loudly at startup with
# the apt line to fix it, instead of failing here for everyone.
#   sudo apt install libphidget22-dev
find_path(PHIDGET22_INCLUDE_DIR phidget22.h)
find_library(PHIDGET22_LIBRARY NAMES phidget22)
if(PHIDGET22_INCLUDE_DIR AND PHIDGET22_LIBRARY)
  message(STATUS "imu_dds: Phidget22 found (${PHIDGET22_LIBRARY}) — IMU.Source=\"phidget\" enabled")
  LIST(APPEND SOURCES ../src/phidget_imu.cpp)
  LIST(APPEND HEADERS ../src/phidget_imu.h)
  LIST(APPEND LIBS ${LIBS} ${PHIDGET22_LIBRARY})
  include_directories(${PHIDGET22_INCLUDE_DIR})
  add_definitions(-DHAVE_PHIDGET22)
else()
  message(WARNING "imu_dds: Phidget22 NOT found — building WITHOUT phidget support "
                  "(IMU.Source=\"phidget\" will refuse to start). Fix: sudo apt install libphidget22-dev")
endif()

include_directories(${MEDIA_TRANSPORT_DIR})

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
