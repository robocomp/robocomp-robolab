#ifndef CYLINDER_H_
#define CYLINDER_H_

#include "axis.h"

// Cylinder class
class Cylinder:public Axis
{
protected:
  double r[2]; //!< Radius and squared radius.
  Vector c;   //!< Center.
  double h;    //!< Half length.
public:
  Cylinder(const Vector&,const Vector&,const double&);
  double R(const Vector&) const;
  void Set(const Vector&,const Vector&);
  double R(const double&) const;
};

#endif