#include <iostream> 
#include "Units.h" 

#ifndef METRICMATH_H
#define METRICMATH_H


namespace metricmath{

    const double PI = 3.141; 

    units::Radian degreeToRadian(units::Degree degree);

    units::Degree radianToDegree(units::Radian radian);

    units::Radian addToOrientation(units::Radian orientation, units::Radian addition);
} //namespace metricmath



#endif 