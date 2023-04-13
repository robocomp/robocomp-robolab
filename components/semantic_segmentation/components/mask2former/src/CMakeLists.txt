cmake_minimum_required(VERSION 2.6)
PROJECT(  Semantic_Segmentation )

INCLUDE( $ENV{ROBOCOMP}/cmake/robocomp.cmake )

ROBOCOMP_IDSL_TO_ICE( CommonBehavior CameraRGBDSimple SemanticSegmentation )
