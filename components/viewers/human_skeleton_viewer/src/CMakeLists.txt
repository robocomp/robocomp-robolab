cmake_minimum_required(VERSION 2.6)
PROJECT(  human_skeleton_viewer )

INCLUDE( /opt/robocomp/cmake/robocomp.cmake )

ROBOCOMP_IDSL_TO_ICE( CommonBehavior CameraRGBDSimple HumanCameraBody YoloObjects )
