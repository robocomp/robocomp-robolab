
#ifndef AXIS_H_XX
#define AXIS_H_XX

#include <vector>
#include "vector.h"


// Axis class
class Axis
{
protected:
  Vector a,b;   //!< End vertices of the axis.
  Vector axis;  //!< Normalized axis vector.
  double length; //!< Length of the axis.
  std::vector<double> quadric; //!< Quadric equation of the squared distance to the axis.
  std::vector<double> linear;  //!< Linear equation of the distance along the axis.
public:
  Axis()
  {
    quadric.resize(3);
    linear.resize(2);
  }
  Axis(const Vector&,const Vector&);
  inline Vector getA () { return a; }
  inline Vector getB () { return b; }
  inline double getLength () { return length; }
  void setAxis(const Vector&,const Vector&);
};

#endif
