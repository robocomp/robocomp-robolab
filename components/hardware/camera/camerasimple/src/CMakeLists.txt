cmake_minimum_required(VERSION 2.6)
PROJECT(  camerasimple )

INCLUDE( $ENV{ROBOCOMP}/cmake/robocomp.cmake )

ROBOCOMP_IDSL_TO_ICE( CommonBehavior CameraSimple )
