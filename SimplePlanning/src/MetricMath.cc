#include "MetricMath.h"


namespace metricmath{

    // const double PI = 3.141; 


    units::Radian degreeToRadian(units::Degree degree){
        return units::Radian{degree.value * PI / 180.0}; 
    }

    units::Degree radianToDegree(units::Radian radian){
        return units::Degree{radian.value * 180.0 / PI} ; 
    }

    units::Radian addToOrientation(units::Radian orientation, units::Radian addition){
        double new_orient = ( (orientation.value + addition.value) ); 
        if ( new_orient >= 2 * PI ){
            new_orient = new_orient - 2*PI; 
        }
        if ( new_orient < 0){
            new_orient = new_orient + 2 * PI; 
        }
        return units::Radian{new_orient};
    }
} //namespace metricmath
