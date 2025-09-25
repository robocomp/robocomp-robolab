cmake_minimum_required(VERSION 3.10)
PROJECT(  python_xbox_controller )


#Checks Paths
IF ( "$ENV{ROBOCOMP}" STREQUAL "")
  MESSAGE(WARNING "ROBOCOMP variable not set. Using the default value: /home/robocomp/robocomp")
  SET (ENV{ROBOCOMP} "/home/robocomp/robocomp/")
ENDIF ( "$ENV{ROBOCOMP}" STREQUAL "")

IF(NOT EXISTS $ENV{ROBOCOMP}/cmake)
  MESSAGE(FATAL_ERROR "Couldn't find RoboComp in $ENV{ROBOCOMP}! Exiting...")
ENDIF(NOT EXISTS $ENV{ROBOCOMP}/cmake)

INCLUDE( $ENV{ROBOCOMP}/cmake/robocomp.cmake )


SET(RC_COMPONENT_PATH ${CMAKE_CURRENT_SOURCE_DIR} CACHE STRING "component base path")
SET(RC_COMPONENT_DEVEL_PATH ${CMAKE_CURRENT_SOURCE_DIR} CACHE STRING "component binary path")
SET(RC_OUTPUT_ICE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/generated)

#Generate ices
ROBOCOMP_IDSL_TO_ICE(JoystickAdapter JoystickAdapter )


ADD_SUBDIRECTORY( src )


MACRO( WRAP_PYTHON_UI )
  FOREACH( input_file ${ARGN} )
    ADD_CUSTOM_COMMAND (
      OUTPUT src/ui_${input_file}.py
      COMMAND pyside6-uic ${CMAKE_CURRENT_SOURCE_DIR}/src/${input_file}.ui -o ${CMAKE_CURRENT_SOURCE_DIR}/src/ui_${input_file}.py
      DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/${input_file}.ui
      COMMENT "Generating src/ui_${input_file}.py from src/${input_file}.ui"
    )
    ADD_CUSTOM_TARGET(ui_${input_file} ALL DEPENDS src/ui_${input_file}.py )
  ENDFOREACH( input_file )
ENDMACRO( WRAP_PYTHON_UI )


#Add symbolic link bin
add_custom_target(create_symlink ALL
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_SOURCE_DIR}/bin
  COMMAND ${CMAKE_COMMAND} -E create_symlink
    ${CMAKE_CURRENT_SOURCE_DIR}/generated/python_xbox_controller.py
    ${CMAKE_CURRENT_SOURCE_DIR}/bin/python_xbox_controller
  COMMENT "Creando enlace simbólico en bin/"
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/generated/python_xbox_controller.py )
