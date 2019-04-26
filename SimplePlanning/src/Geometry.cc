#include "Geometry.h" 
/**
 * Geometry: 
 * Note: 
 * Coordinate Axis; y( points upward), x(points to the right) 
 * GridCoordinate:  rows (points downward), columns(points to the right) 
 * 
 * 
 * */

namespace geometry{

RectangleGeometry::RectangleGeometry(){}

RectangleGeometry::RectangleGeometry(Point topleft, Dimension dimension):
_topleft(topleft),_dimension(dimension) {
}    

BoundingBox RectangleGeometry::getBoundingBox() const{
    Point bottom_right {_topleft.x.value + _dimension.width.value,
                        _topleft.y.value - _dimension.height.value}; 

    BoundingBox box{_topleft, bottom_right};
    return box;
}

}

