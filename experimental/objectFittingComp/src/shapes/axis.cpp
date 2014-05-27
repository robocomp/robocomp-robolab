// Axis
// Changelog 01.12.17
#include <stdio.h>
#include "axis.h"

/*!
  \class Axis 
  \brief This class implements a minimal data-structure to define 
  a simple axis characterized by its end vertices.
*/

/*!
  \brief Creates a generic axis given end vertices.
  \param a, b End vertices of the axis.
*/
Axis::Axis(const Vector& a,const Vector& b)
{
  Axis::a=a;
  Axis::b=b; 
  axis=b-a;
//   printf("   a(%f, %f, %f)  b(%f, %f, %f)", a[0], a[1], a[2], b[0], b[1], b[2]);
//   printf("axis(%f, %f, %f)", axis[0], axis[1], axis[2]);
  length=Norm(axis);
//   printf("length:%g\n", length);
  axis/=length;
}

void Axis::setAxis(const Vector& a,const Vector& b)
{
  Axis::a=a;
  Axis::b=b; 
  axis=b-a;
//   printf("   a(%f, %f, %f)  b(%f, %f, %f)", a[0], a[1], a[2], b[0], b[1], b[2]);
//   printf("axis(%f, %f, %f)", axis[0], axis[1], axis[2]);
  length=Norm(axis);
//   printf("length:%g\n", length);
  axis/=length;
}