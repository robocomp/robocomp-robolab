
#ifndef AXIS_H_
#define AXIS_H_

#include "vector.h"

// Axis class
class Axis
{
protected:
  Vector a,b;   //!< End vertices of the axis.
  Vector axis;  //!< Normalized axis vector.
  double length; //!< Length of the axis.
  double quadric[3]; //!< Quadric equation of the squared distance to the axis.
  double linear[2];  //!< Linear equation of the distance along the axis.
public:
  Axis(const Vector&,const Vector&);
};

#endif AXIS_H_
