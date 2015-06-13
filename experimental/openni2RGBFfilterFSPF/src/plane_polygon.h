//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    plane_polygon.h
\brief   C++ Interfaces: PlanePolygon
\author  Joydeep Biswas, (C) 2010
*/
//========================================================================

#include <vector>
#include <fstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <xmmintrin.h>
#include <smmintrin.h>
#include "geometry.h"
#include "quaternion_helper.h"
#include "terminal_utils.h"
#include "timer.h"
#include "grahams_scan.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/Eigen/Geometry>

#ifndef PLANE_POLYGON_H
#define PLANE_POLYGON_H

using namespace std;
using namespace Eigen;


/**
A class used to reprsesent a convex plane polygon with sampled points
**/
class PlanePolygon{
protected:
  /// Normals to the edges. normals2D[i] is normal to the edge (vertices2D[i], vertices2D[i+1])
  vector<vector2d> normals2D; 
  /// Unit vectors parallel to the edges. edgeDir2D[i] is parallel to the edge (vertices2D[i], vertices2D[i+1])
  vector<vector2d> edgeDir2D; 
  /// Length of the edge (vertices2D[i], vertices2D[i+1])
  vector<double> edgeLengths; 
  /// Offsets to the edges such that for an interior point p, normals2D[i].dot(p)+offsets2D[i]> 0 for all i.
  vector<double> offsets2D;

  /// Graham Scan class used to generate convex hull of the polygon
  GrahamsScan grahamsScan;
  /// Matrices for computing optimal plane parameters
  Matrix3d m, eigenVectors;
  /// Eigenvectors of the plane scatter matrix
  Vector3d eigenValues;
  /// Solver to solve for optimal plane parameters
  SelfAdjointEigenSolver<Matrix3d> solver;

public:
  /// Number of points used to build the polygon
  double numPoints;
  /// Pixel locations of the points sampled from the depth image used to construct the polygon
  vector<vector2i> pixelLocs; 
  /// Vertices of the polygon
  vector<vector3f> vertices; 
  /// Vertices projected onto the 2D basis vectors
  vector<vector2f> vertices2D;
  // Plane equation: for points p on the plane, (p-p0).dot(normal) = 0 or p.dot(normal) + offset = 0
  /// Normal to the plane
  vector3f normal; 
  /// Point on the plane corresponding to the centroid of the sampled points
  vector3f p0; 
  /// Perpendicular offset of the plane from the origin
  double offset;
  /// Basis vectors on the plane. b1 corresponds to the major axis and b2 to the minor axis
  vector3f b1, b2; 

  
  /// Coefficients of the moments of the points
  double sum_xx, sum_yy, sum_zz, sum_xy, sum_xz, sum_yz;
  /// Rectangular dimensions of the polygon
  float width, height;
  /// Rectangular extents of the polygon in 2D
  vector2f min2D, max2D;
  /// Rectangular extents of the polygon in 3D
  vector3f corners[4];
  
  /// Indicates whether a valid convex plane polygon fit was copmuted or not.
  bool validPolygon;
  
  /// Ratio of the eigenvalues corresponding to the planar basis vectors
  double conditionNumber;
  
protected:
  /// Construct boindary points to make a convex polygon
  bool constructConvexPoly(vector<vector3f>& points);
  /// Compute Plane normal, offset and scatter matrix coefficients from given points
  bool computePlaneParameters(vector<vector3f>& points);
  
public:
  /// Empty (default) constructor has nothing to do
  PlanePolygon();
  /// Build polygon using given plane filtered points
  PlanePolygon(vector< vector3f > points);
  /// Build polygon using given plane filtered points, and also saves the pixel locations
  PlanePolygon(vector<vector3f> points, vector<vector2i> _pixelLocs);
  /// Destructor
  ~PlanePolygon();
  
  /// Returns distance of specified point from the plane
  double distFromPlane(vector3f p);
  /// Returns ray perpendicular to the plane and pointing from the plane to the specified point p
  vector3f rayFromPlane(vector3f p);
  /// Project 3D point onto the plane polygon and return corresponding 2D coordinates
  vector2f projectOnto (const vector3f& p) const {return vector2f(b1.dot(p-p0), b2.dot(p-p0));} 
  /// Returns true if the point, projected onto the plane, lies within the convex hull
  bool liesAlongside(const vector3f& p) const;
  /// Merge this plane polygon with the one provided
  void merge(PlanePolygon& poly2);
  /// Merge this plane polygon with a set of other polygons
  void merge(vector< PlanePolygon >& polygons);
  /// Transforms the plane polygon by the specified translation and rotation
  void transform(vector3f translation, Quaternionf rotation);
  /// Transforms the plane polygon by the specified transformation matrix
  void transform(const GVector::matrix3d< float >& M);
  /// Returns the point of intersection of the line =d*l+l0 and this plane
  vector3f intersect(vector3f l0, vector3f l);
};

#endif //PLANE_POLYGON_H
