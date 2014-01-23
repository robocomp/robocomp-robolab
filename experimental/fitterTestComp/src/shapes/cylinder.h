#ifndef CYLINDER_H_
#define CYLINDER_H_

#include <vector>
#include "axis.h"

// Cylinder class
class Cylinder: public Axis
{
public:
// protected:
  std::vector<double> r; //!< Radius and squared radius.
  Vector c;   //!< Center.
  double h;    //!< Half length.
public:
  Cylinder()
  {
    r.resize(2);
  }
  Cylinder(const Vector&, const Vector&, const double&);
  inline double getR () { return r[0]; }
  void setValues(const Vector&, const Vector&, const double&);
  double R(const Vector&) const;
  void Set(const Vector&, const Vector&);
  double R(const double&) const;
};

#endif
