cmake_minimum_required(VERSION 2.6)
PROJECT(  pioneer_viewer_py )

INCLUDE( /home/robocomp/robocomp/cmake/robocomp.cmake )

ROBOCOMP_IDSL_TO_ICE( CommonBehavior BatteryStatus CameraRGBDSimple DifferentialRobot GenericBase )
