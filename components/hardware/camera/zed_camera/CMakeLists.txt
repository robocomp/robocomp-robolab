cmake_minimum_required(VERSION 3.10)

# Forzar g++-14 (soporte C++23 <print>, no disponible en gcc-13 de noble)
if(NOT CMAKE_CXX_COMPILER)
    find_program(GXX14 g++-14)
    if(GXX14)
        set(CMAKE_CXX_COMPILER ${GXX14} CACHE FILEPATH "C++ compiler" FORCE)
    endif()
endif()

if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()

project(zed_camera)

# Ruta del Toolkit CUDA
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda" CACHE PATH "CUDA Toolkit")
set(CUDA_ROOT "/usr/local/cuda")
set(CUDAToolkit_ROOT "/usr/local/cuda")

# Ayuda a FindCUDA a localizar nvcc
set(CUDA_NVCC_EXECUTABLE "${CUDA_TOOLKIT_ROOT_DIR}/bin/nvcc" CACHE FILEPATH "CUDA compiler" FORCE)

IF ( "$ENV{ROBOCOMP}" STREQUAL "")
  MESSAGE(WARNING "ROBOCOMP variable not set. Using the default value: /home/robocomp/robocomp")
  SET (ENV{ROBOCOMP} "/home/robocomp/robocomp/")
ENDIF ( "$ENV{ROBOCOMP}" STREQUAL "")

SET(RC_COMPONENT_PATH ${CMAKE_CURRENT_SOURCE_DIR} CACHE STRING "component base path" FORCE)
SET(RC_COMPONENT_DEVEL_PATH "${CMAKE_CURRENT_SOURCE_DIR}" CACHE STRING "component binary path" FORCE)
#SET(RC_COMPONENT_DEVEL_PATH "${WORKSPACE_PATH}/devel"  CACHE STRING "component binary path")
SET(RC_COMPONENT_INSTALL_PATH $ENV{ROBOCOMP} CACHE STRING "component install path")

#commented as now devel is merged into source space
#get_filename_component( COMPONENT_NAME ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
#STRING(REPLACE "${COMPONENT_NAME}/" " " COMPONENT_NAME  ${RC_COMPONENT_PATH})
#STRING(STRIP ${COMPONENT_NAME} COMPONENT_NAME)
#SET(COMPONENT_NAME ${COMPONENT_NAME} CACHE STRING "component name" FORCE)

add_subdirectory(generated)

INSTALL(FILES etc/config DESTINATION ${RC_COMPONENT_INSTALL_PATH}/etc-default/ RENAME zed_camera.conf )

#ADD_CUSTOM_TARGET(doc doxygen Doxyfile)
#ADD_CUSTOM_TARGET(installdoc mkdir -p ${RC_COMPONENT_INSTALL_PATH}/doc COMMAND cp -R doc/html ${RC_COMPONENT_INSTALL_PATH}/doc/zed_camera )
