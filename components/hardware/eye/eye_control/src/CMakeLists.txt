cmake_minimum_required(VERSION 2.6)
PROJECT(  eye_control )

INCLUDE( /opt/robocomp/cmake/robocomp.cmake )

ROBOCOMP_IDSL_TO_ICE( CommonBehavior CameraRGBDSimple DifferentialRobot GenericBase HumanCameraBody JointMotorSimple SoundRotation )
