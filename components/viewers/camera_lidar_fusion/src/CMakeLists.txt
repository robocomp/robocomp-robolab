cmake_minimum_required(VERSION 2.6)
PROJECT(  camera_lidar_fusion )

INCLUDE( /home/robocomp/robocomp/cmake/robocomp.cmake )

ROBOCOMP_IDSL_TO_ICE( CommonBehavior Camera360RGB Lidar3D )
