cmake_minimum_required(VERSION 2.6)
PROJECT(  pioneer_viewer )

INCLUDE( /home/robocomp/robocomp/cmake/robocomp.cmake )

ROBOCOMP_IDSL_TO_ICE( CommonBehavior CameraRGBDSimple )
