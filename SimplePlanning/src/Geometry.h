#include "Units.h"

#ifndef GEOMETRY_H
#define GEOMETRY_H

namespace geometry{
/**
 * Geometry: 
 * Note: 
 * Coordinate Axis; y( points upward), x(points to the right) 
 * GridCoordinate:  rows (points downward), columns(points to the right) 
 * 
 * 
 * */



    struct Point{
        /**
         * y (points upward), x (points to the right )
         * */
        units::Meter x; 
        units::Meter y; 
    };

    struct BoundingBox{
        Point topLeft; 
        Point bottomRight; 
    };    

    struct Dimension{
        units::Meter width; 
        units::Meter height; 
    };

    class Geometry{
        public: 
            Geometry(){};
            virtual BoundingBox getBoundingBox() const = 0; 
    };


    class RectangleGeometry : public Geometry{
        public: 
            RectangleGeometry();
            RectangleGeometry(Point topleft_corner,Dimension dimension); 
            BoundingBox getBoundingBox() const;

        private:
            Point _topleft; 
            Dimension _dimension;      
    };

}


#endif 
