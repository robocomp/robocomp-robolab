cmake_minimum_required(VERSION 2.6)
PROJECT(  FacialEmotionRecognition )

INCLUDE( $ENV{ROBOCOMP}/cmake/robocomp.cmake )

ROBOCOMP_IDSL_TO_ICE( CommonBehavior CameraSimple )
