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
\file    vector_map3d.cpp
\brief   C++ Implementation: VectorMap3D
\author  Joydeep Biswas, (C) 2010
*/
//========================================================================

#include "plane_polygon.h"

//============================================================================================

#define DEBUG_FUNCTION_CALLS() printf("%s @ %s:%d\n",__FUNCTION__,__FILE__,__LINE__);

static const bool debugConstructors = false;

PlanePolygon::PlanePolygon() : normals2D(0), edgeDir2D(0), edgeLengths(0), offsets2D(0), pixelLocs(0), vertices(0), vertices2D(0) 
{
  if(debugConstructors) printf("start of %s\n",__PRETTY_FUNCTION__);
  normal.zero();
  offset = 0.0;
  b1.zero();
  b2.zero();
  p0.zero();
  numPoints = 0.0;
  validPolygon = false;
}

PlanePolygon::PlanePolygon(vector< vector3f > points) : normals2D(0), edgeDir2D(0), edgeLengths(0), offsets2D(0), pixelLocs(0), vertices(0), vertices2D(0) 
{
  if(debugConstructors) printf("start of %s\n",__PRETTY_FUNCTION__);
  numPoints = points.size();
  validPolygon = computePlaneParameters(points);
  if(validPolygon)
    validPolygon = constructConvexPoly(points);
  pixelLocs.clear();
  if(debugConstructors) printf("end of %s\n",__PRETTY_FUNCTION__);
}

PlanePolygon::PlanePolygon(vector< vector3f > points, vector< vector2i > _pixelLocs) : normals2D(0), edgeDir2D(0), edgeLengths(0), offsets2D(0), pixelLocs(0), vertices(0), vertices2D(0)
{
  if(debugConstructors) printf("start of %s\n",__PRETTY_FUNCTION__);
  numPoints = points.size();
  validPolygon = computePlaneParameters(points);
  if(validPolygon)
    validPolygon = constructConvexPoly(points);
  pixelLocs = _pixelLocs;
  if(debugConstructors) printf("end of %s\n",__PRETTY_FUNCTION__);
}

PlanePolygon::~PlanePolygon()
{
  if(debugConstructors) printf("start of %s\n",__PRETTY_FUNCTION__);
  normals2D.clear(); 
  edgeDir2D.clear(); 
  edgeLengths.clear();
  offsets2D.clear();
  pixelLocs.clear();
  vertices.clear(); 
  vertices2D.clear(); 
  if(debugConstructors) printf("end of %s\n",__PRETTY_FUNCTION__);
}

bool PlanePolygon::computePlaneParameters(vector< vector3f >& points)
{
  static const bool debug = true;
  using namespace Eigen;
  static Matrix3d m, eigenVectors;
  static Vector3d eigenValues;
  static SelfAdjointEigenSolver<Matrix3d> solver;
  
  vector3f p;
  double n = double(points.size());
  
  sum_xx = 0.0;
  sum_yy = 0.0;
  sum_zz = 0.0;
  sum_xy = 0.0;
  sum_xz = 0.0;
  sum_yz = 0.0;
  unsigned int i;
  
  p0.zero();
  for(i=0; i<points.size(); i++){
    p0 += points[i];
  }
  p0/=n;
  // Compute statistics required to construct the Scatter Matrix
  for(i=0; i<points.size(); i++){
    p = points[i];
    sum_xx += sq(p.x);
    sum_yy += sq(p.y);
    sum_zz += sq(p.z);
    sum_xy += p.x*p.y;
    sum_xz += p.x*p.z;
    sum_yz += p.y*p.z;
  }
  
  double invN = 1.0/n;
  
  m(0,0) = invN*sum_xx - p0.x*p0.x;
  m(0,1) = invN*sum_xy - p0.x*p0.y;
  m(0,2) = invN*sum_xz - p0.x*p0.z;
  m(1,0) = invN*sum_xy - p0.x*p0.y;
  m(1,1) = invN*sum_yy - p0.y*p0.y;
  m(1,2) = invN*sum_yz - p0.y*p0.z;
  m(2,0) = invN*sum_xz - p0.x*p0.z;
  m(2,1) = invN*sum_yz - p0.y*p0.z;
  m(2,2) = invN*sum_zz - p0.z*p0.z;
  solver.compute(m);
  eigenVectors = solver.eigenvectors();
  eigenValues = solver.eigenvalues();
  
  double minEig=0, midEig=0, maxEig=0;
  int minInd=-1, midInd=-1, maxInd=-1;
  for(int i=0; i<3; i++){
    if(minEig>eigenValues(i) || minInd<0){
      minEig = eigenValues(i);
      minInd = i;
    }
    if(maxEig<eigenValues(i) || maxInd<0){
      maxEig = eigenValues(i);
      maxInd = i;
    }
  }
  
  midInd = 3 - (minInd+maxInd);
  midEig = eigenValues(midInd);
  
  conditionNumber = midEig/maxEig;
  
  if(minInd<0 || maxInd<0 || maxInd==minInd){
    // Ill-defined plane
    normal.zero();
    offset = 0.0;
    b1.zero();
    b2.zero();
    p0.zero();
    numPoints = 0.0;
    if(debug) printf("Ill defined plane!\n");
    return false;
  }
  
  
  if(debug && false){
    printf("Planar points:\n");
    for(i=0; i<points.size(); i++){
      printf("%f,%f,%f\n",V3COMP(points[i]));
    }
    printf("\nEigenvalues order: %d %d %d\n",minInd,midInd,maxInd);
    printf("\nCentroid: %f,%f,%f\n",V3COMP(p0));
    cout<<"\nMatrix:\n"<<m<<"\n\nEigenValues:\n"<<eigenValues<<"\n\nEigenVectors:\n"<<eigenVectors<<"\n\n";
  }
  
  n = n;
  normal.set(eigenVectors(0,minInd), eigenVectors(1,minInd), eigenVectors(2,minInd),0.0);
  normal.normalize();
  if(normal.dot(p0)<0.0)
    normal = -normal;  
  offset = -normal.dot(p0);
  b1.set(eigenVectors(0,maxInd), eigenVectors(1,maxInd), eigenVectors(2,maxInd),0.0);
  b2.set(eigenVectors(0,midInd), eigenVectors(1,midInd), eigenVectors(2,midInd),0.0);
  sum_xx = invN*sum_xx;
  sum_yy = invN*sum_yy;
  sum_zz = invN*sum_zz;
  sum_xy = invN*sum_xy;
  sum_xz = invN*sum_xz;
  sum_yz = invN*sum_yz;
  return true;
}

bool PlanePolygon::constructConvexPoly(vector< vector3f >& points)
{
  static const bool debug = true;
  if(points.size()<3){
    if(debug) printf("Too few points!\n");
    return false; //Can't construct a polygon with less than 3 vertices!
  }
  
  vector<vector2f> &points2d = vertices2D;
  points2d.clear();
  
  for(unsigned int i=0; i<points.size(); i++){
    vector2f p(b1.dot(points[i]-p0), b2.dot(points[i]-p0));
    points2d.push_back(p);
  }
  vertices2D = grahamsScan.run(points2d);
  
  unsigned int N = vertices2D.size();
  normals2D.resize(N);
  offsets2D.resize(N);
  vertices.resize(N);
  
  vector3f p;
  min2D.setAll(FLT_MAX);
  max2D.setAll(-FLT_MAX);
  for(unsigned int i=0; i<N; i++){
    p = vertices2D[i].x*b1 + vertices2D[i].y*b2 + p0;
    min2D.x = min(min2D.x,vertices2D[i].x);
    min2D.y = min(min2D.y,vertices2D[i].y);
    max2D.x = max(max2D.x,vertices2D[i].x);
    max2D.y = max(max2D.y,vertices2D[i].y);
    vertices[i] = p;
    normals2D[i] = (vertices2D[(i+1)%N]-vertices2D[i]).perp().norm();
    offsets2D[i] = -normals2D[i].dot(vertices2D[i]);
    if(normals2D[i].dot(vertices2D[(i+2)%N])+offsets2D[i]<0.0){
      normals2D[i] = -normals2D[i];
      offsets2D[i] = -offsets2D[i];
    }
  }
  corners[0] = p0 + b1*min2D.x + b2*min2D.y;
  corners[1] = p0 + b1*max2D.x + b2*max2D.y;
  corners[2] = p0 + b1*max2D.x + b2*min2D.y;
  corners[3] = p0 + b1*min2D.x + b2*max2D.y;
  width = 0.5*(max2D.x - min2D.x);
  height = 0.5*(max2D.y - min2D.y);
  if(width<=0.0 || height<=0.0){
    if(debug) printf("zero size polygon! width: %f, height:%f\n",width, height);
    return false;
  }
  return true;
}

double PlanePolygon::distFromPlane(vector3f p)
{
  return fabs(p.dot(normal) + offset);
}

vector3f PlanePolygon::rayFromPlane(vector3f p)
{
  return normal*(p.dot(normal)+offset);
}


bool PlanePolygon::liesAlongside(const vector3f& p) const
{
  vector2f p2 = projectOnto(p);
  bool interior = true;
  if(normals2D.size() != offsets2D.size() || offsets2D.size() != vertices2D.size()){
    printf("Vector size mismatch in %s normals2D.size():%d offsets2D.size():%d vertices2D.size():%d\n",__PRETTY_FUNCTION__,int(normals2D.size()), int(offsets2D.size()), int(vertices2D.size()));
    exit(1);
  }
  for(unsigned int i=0; i<normals2D.size() && interior; i++){
    interior = (normals2D[i].dot(p2)+offsets2D[i])>0.0;
  }
  return interior;
}

void PlanePolygon::merge(PlanePolygon& poly2)
{  
  static const bool debug = false;
  using namespace Eigen;
  
  double newNumPoints = numPoints + poly2.numPoints;
  vector3d p0Double = (numPoints*vector3d(V3COMP(p0)) + poly2.numPoints*vector3d(V3COMP(poly2.p0)))/newNumPoints;
  
  // Compute statistics required to construct the MI tensor
  
  double weight1 = numPoints/newNumPoints, weight2 = poly2.numPoints/newNumPoints;
  
  sum_xx = weight1*sum_xx + weight2*poly2.sum_xx;
  sum_yy = weight1*sum_yy + weight2*poly2.sum_yy;
  sum_zz = weight1*sum_zz + weight2*poly2.sum_zz;
  sum_xy = weight1*sum_xy + weight2*poly2.sum_xy;
  sum_xz = weight1*sum_xz + weight2*poly2.sum_xz;
  sum_yz = weight1*sum_yz + weight2*poly2.sum_yz;
  
  m(0,0) = sum_xx - p0Double.x*p0Double.x;
  m(0,1) = sum_xy - p0Double.x*p0Double.y;
  m(0,2) = sum_xz - p0Double.x*p0Double.z;
  m(1,0) = sum_xy - p0Double.x*p0Double.y;
  m(1,1) = sum_yy - p0Double.y*p0Double.y;
  m(1,2) = sum_yz - p0Double.y*p0Double.z;
  m(2,0) = sum_xz - p0Double.x*p0Double.z;
  m(2,1) = sum_yz - p0Double.y*p0Double.z;
  m(2,2) = sum_zz - p0Double.z*p0Double.z;
  
  solver.compute(m);
  eigenVectors = solver.eigenvectors();
  eigenValues = solver.eigenvalues();
  
  double minEig=0, maxEig=0;
  int minInd=-1, midInd=-1, maxInd=-1;
  for(int i=0; i<3; i++){
    if(minEig>eigenValues(i) || minInd<0){
      minEig = eigenValues(i);
      minInd = i;
    }
    if(maxEig<eigenValues(i) || maxInd<0){
      maxEig = eigenValues(i);
      maxInd = i;
    }
  }
  
  midInd = 3 - (minInd+maxInd);
  
  if(debug){
    printf("\nEigenvalues order: %d %d %d\n",minInd,midInd,maxInd);
    printf("\nCentroid: %f,%f,%f\n",V3COMP(p0Double));
    std::cout<<"\nMatrix:\n"<<m<<"\n\nEigenValues:\n"<<eigenValues<<"\n\nEigenVectors:\n"<<eigenVectors<<"\n\n";
    //exit(0);
  }
  
  numPoints = newNumPoints;
  normal.set(eigenVectors(0,minInd), eigenVectors(1,minInd), eigenVectors(2,minInd),0.0);
  normal.normalize();
  p0.set(V3COMP(p0Double));
  if(normal.dot(p0) <0.0)
    normal = -normal;
  offset = -normal.dot(p0);
  b1.set(eigenVectors(0,maxInd), eigenVectors(1,maxInd), eigenVectors(2,maxInd),0.0);
  b2.set(eigenVectors(0,midInd), eigenVectors(1,midInd), eigenVectors(2,midInd),0.0);
  
  vector<vector2f> points2d;
  points2d.clear();
  
  for(unsigned int i=0; i<vertices.size(); i++){
    vector2f p(b1.dot(vertices[i]-p0), b2.dot(vertices[i]-p0));
    points2d.push_back(p);
  }
  for(unsigned int i=0; i<poly2.vertices.size(); i++){
    vector2f p(b1.dot(poly2.vertices[i]-p0), b2.dot(poly2.vertices[i]-p0));
    points2d.push_back(p);
  }
  
  vertices2D = grahamsScan.run(points2d);
  
  unsigned int N = vertices2D.size();
  normals2D.resize(N);
  offsets2D.resize(N);
  vertices.resize(N);
  vector3f p;
  min2D.setAll(FLT_MAX);
  max2D.setAll(-FLT_MAX);
  for(unsigned int i=0; i<vertices2D.size(); i++){
    p = vertices2D[i].x*b1 + vertices2D[i].y*b2 + p0;
    min2D.x = min(min2D.x,vertices2D[i].x);
    min2D.y = min(min2D.y,vertices2D[i].y);
    max2D.x = max(max2D.x,vertices2D[i].x);
    max2D.y = max(max2D.y,vertices2D[i].y);
    vertices[i] = p;
    normals2D[i] = (vertices2D[(i+1)%N]-vertices2D[i]).perp().norm();
    offsets2D[i] = -normals2D[i].dot(vertices2D[i]);
    if(normals2D[i].dot(vertices2D[(i+2)%N])+offsets2D[i]<0.0){
      normals2D[i] = -normals2D[i];
      offsets2D[i] = -offsets2D[i];
    }
  }
  corners[0] = p0 + b1*min2D.x + b2*min2D.y;
  corners[1] = p0 + b1*max2D.x + b2*max2D.y;
  corners[2] = p0 + b1*max2D.x + b2*min2D.y;
  corners[3] = p0 + b1*min2D.x + b2*max2D.y;
  width = 0.5*(max2D.x - min2D.x);
  height = 0.5*(max2D.y - min2D.y);
  
  if(normals2D.size() != offsets2D.size() || offsets2D.size() != vertices2D.size()){
    printf("normals2D.size():%d offsets2D.size():%d vertices2D.size():%d\n",int(normals2D.size()), int(offsets2D.size()), int(vertices2D.size()));
    exit(3);
  }
}

void PlanePolygon::merge(vector< PlanePolygon >& polygons)
{
  static const bool debug = false;
  using namespace Eigen;
  
  double newNumPoints = numPoints;
  vector3d p0Double = numPoints*vector3d(V3COMP(p0));
  for(unsigned int i=0; i<polygons.size(); i++){
    newNumPoints += polygons[i].numPoints;
    p0Double += polygons[i].numPoints*vector3d(V3COMP(polygons[i].p0));
  }
  p0Double = p0Double/newNumPoints;
  
  // Compute statistics required to construct the MI tensor
  
  double weight = numPoints/newNumPoints;
  
  sum_xx = weight*sum_xx;
  sum_xy = weight*sum_xy;
  sum_xz = weight*sum_xz;
  sum_yy = weight*sum_yy;
  sum_yz = weight*sum_yz;
  sum_zz = weight*sum_zz;
  
  for(unsigned int i=0; i<polygons.size(); i++){
    weight = polygons[i].numPoints/newNumPoints;
    sum_xx += weight*polygons[i].sum_xx;
    sum_xy += weight*polygons[i].sum_xy;
    sum_xz += weight*polygons[i].sum_xz;
    sum_yy += weight*polygons[i].sum_yy;
    sum_yz += weight*polygons[i].sum_yz;
    sum_zz += weight*polygons[i].sum_zz;
  }
  
  m(0,0) = sum_xx - p0Double.x*p0Double.x;
  m(0,1) = sum_xy - p0Double.x*p0Double.y;
  m(0,2) = sum_xz - p0Double.x*p0Double.z;
  m(1,0) = sum_xy - p0Double.x*p0Double.y;
  m(1,1) = sum_yy - p0Double.y*p0Double.y;
  m(1,2) = sum_yz - p0Double.y*p0Double.z;
  m(2,0) = sum_xz - p0Double.x*p0Double.z;
  m(2,1) = sum_yz - p0Double.y*p0Double.z;
  m(2,2) = sum_zz - p0Double.z*p0Double.z;
  
  solver.compute(m);
  eigenVectors = solver.eigenvectors();
  eigenValues = solver.eigenvalues();
  
  double minEig=0, maxEig=0;
  int minInd=-1, midInd=-1, maxInd=-1;
  for(int i=0; i<3; i++){
    if(minEig>eigenValues(i) || minInd<0){
      minEig = eigenValues(i);
      minInd = i;
    }
    if(maxEig<eigenValues(i) || maxInd<0){
      maxEig = eigenValues(i);
      maxInd = i;
    }
  }
  
  midInd = 3 - (minInd+maxInd);
  
  if(debug){
    printf("\nEigenvalues order: %d %d %d\n",minInd,midInd,maxInd);
    printf("\nCentroid: %f,%f,%f\n",V3COMP(p0Double));
    std::cout<<"\nMatrix:\n"<<m<<"\n\nEigenValues:\n"<<eigenValues<<"\n\nEigenVectors:\n"<<eigenVectors<<"\n\n";
    //exit(0);
  }
  
  numPoints = newNumPoints;
  normal.set(eigenVectors(0,minInd), eigenVectors(1,minInd), eigenVectors(2,minInd),0.0);
  normal.normalize();
  p0.set(V3COMP(p0Double));
  if(normal.dot(p0) <0.0)
    normal = -normal;
  offset = -normal.dot(p0);
  b1.set(eigenVectors(0,maxInd), eigenVectors(1,maxInd), eigenVectors(2,maxInd),0.0);
  b2.set(eigenVectors(0,midInd), eigenVectors(1,midInd), eigenVectors(2,midInd),0.0);
  
  vector<vector2f> points2d;
  points2d.clear();
  
  for(unsigned int i=0; i<vertices.size(); i++){
    vector2f p(b1.dot(vertices[i]-p0), b2.dot(vertices[i]-p0));
    points2d.push_back(p);
  }
  for(unsigned int i=0; i<polygons.size(); i++){
    for(unsigned int j=0; j<polygons[i].vertices.size(); j++){
      vector2f p(b1.dot(polygons[i].vertices[j]-p0), b2.dot(polygons[i].vertices[j]-p0));
      points2d.push_back(p);
    }
  }
  
  vertices2D = grahamsScan.run(points2d);
  
  unsigned int N = vertices2D.size();
  normals2D.resize(N);
  offsets2D.resize(N);
  vertices.resize(N);
  vector3f p;
  min2D.setAll(FLT_MAX);
  max2D.setAll(-FLT_MAX);
  for(unsigned int i=0; i<vertices2D.size(); i++){
    p = vertices2D[i].x*b1 + vertices2D[i].y*b2 + p0;
    min2D.x = min(min2D.x,vertices2D[i].x);
    min2D.y = min(min2D.y,vertices2D[i].y);
    max2D.x = max(max2D.x,vertices2D[i].x);
    max2D.y = max(max2D.y,vertices2D[i].y);
    vertices[i] = p;
    normals2D[i] = (vertices2D[(i+1)%N]-vertices2D[i]).perp().norm();
    offsets2D[i] = -normals2D[i].dot(vertices2D[i]);
    if(normals2D[i].dot(vertices2D[(i+2)%N])+offsets2D[i]<0.0){
      normals2D[i] = -normals2D[i];
      offsets2D[i] = -offsets2D[i];
    }
  }
  corners[0] = p0 + b1*min2D.x + b2*min2D.y;
  corners[1] = p0 + b1*max2D.x + b2*max2D.y;
  corners[2] = p0 + b1*max2D.x + b2*min2D.y;
  corners[3] = p0 + b1*min2D.x + b2*max2D.y;
  width = 0.5*(max2D.x - min2D.x);
  height = 0.5*(max2D.y - min2D.y);
  
  if(normals2D.size() != offsets2D.size() || offsets2D.size() != vertices2D.size()){
    printf("normals2D.size():%d offsets2D.size():%d vertices2D.size():%d\n",int(normals2D.size()), int(offsets2D.size()), int(vertices2D.size()));
    exit(3);
  }
}

void PlanePolygon::transform(vector3f translation, Quaternionf rotation)
{
  GVector::matrix3d<float> M;
  GVector::transformMatrix<float>(rotation, translation, M);
  transform(M);
}

void PlanePolygon::transform(const GVector::matrix3d< float > &M)
{
  static const bool debug = false;
  Eigen::Matrix3d R;
  Eigen::Vector3d t(M.m14,M.m24,M.m34),p(p0.x,p0.y,p0.z);
  
  if(debug){
    printf("\nTransform Matrix:\n");
    printf("%.6f %.6f %.6f %.6f \n",M.m11,M.m12,M.m13,M.m14);
    printf("%.6f %.6f %.6f %.6f \n",M.m21,M.m22,M.m23,M.m24);
    printf("%.6f %.6f %.6f %.6f \n",M.m31,M.m32,M.m33,M.m34);
    printf("%.6f %.6f %.6f %.6f \n",M.m41,M.m42,M.m43,M.m44);
    //exit(0);
    printf("\nbefore:\n");
    printf("p0: %f,%f,%f\n",V3COMP(p0));
    printf("normal: %f,%f,%f\n",V3COMP(normal));
    printf("b1: %f,%f,%f\n",V3COMP(b1));
    printf("b2: %f,%f,%f\n",V3COMP(b2));
    printf("xx:%f xy:%f xz:%f yy:%f yz:%f zz:%f\n",sum_xx,sum_xy,sum_xz,sum_yy,sum_yz,sum_zz);
  }
  // transform points
  for(unsigned int i=0; i<vertices.size(); i++){
    vertices[i] = vertices[i].transform(M);
  }
  
  // transform normals, basis vectors and centroid
  p0 = p0.transform(M);
  normal.w=0.0;
  normal = normal.transform(M);
  b1 = b1.transform(M);
  b2 = b2.transform(M);
  
  // transform scatter matrix elements
  R(0,0) = M.m11;
  R(0,1) = M.m12;
  R(0,2) = M.m13;
  R(1,0) = M.m21;
  R(1,1) = M.m22;
  R(1,2) = M.m23;
  R(2,0) = M.m31;
  R(2,1) = M.m32;
  R(2,2) = M.m33;
  
  m(0,0) = sum_xx;
  m(0,1) = sum_xy;
  m(0,2) = sum_xz;
  m(1,0) = sum_xy;
  m(1,1) = sum_yy;
  m(1,2) = sum_yz;
  m(2,0) = sum_xz;
  m(2,1) = sum_yz;
  m(2,2) = sum_zz;
  
  m = R*m*R.transpose() + t*p.transpose()*R.transpose() + R*p*t.transpose() + t*t.transpose();
  
  sum_xx = m(0,0);
  sum_xy = m(0,1);
  sum_xz = m(0,2);
  sum_yy = m(1,1);
  sum_yz = m(1,2);
  sum_zz = m(2,2);
  
  if(debug){
    printf("\nafter:\n");
    printf("p0: %f,%f,%f\n",V3COMP(p0));
    printf("normal: %f,%f,%f\n",V3COMP(normal));
    printf("b1: %f,%f,%f\n",V3COMP(b1));
    printf("b2: %f,%f,%f\n",V3COMP(b2));
    printf("xx:%f xy:%f xz:%f yy:%f yz:%f zz:%f\n\n",sum_xx,sum_xy,sum_xz,sum_yy,sum_yz,sum_zz);
  }
}

vector3f PlanePolygon::intersect(vector3f l0, vector3f l)
{
  //Implemented from:
  // http://en.wikipedia.org/w/index.php?title=Line-plane_intersection&oldid=458252030#Algebraic_form
  float d = (p0-l0).dot(normal)/l.dot(normal);
  return d*l+l0;
}
