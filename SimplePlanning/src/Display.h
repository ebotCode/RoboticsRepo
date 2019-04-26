#include "Geometry.h"
#include "Map.h"
#include "VehicleModel.h"
#include "MetricMath.h"
#include "Planner.h" 

#include <iostream> 

#ifndef DISPLAY_H
#define DISPLAY_H


namespace display {

    void print(geometry::BoundingBox box);

    void print(geometry::RectangleGeometry rect);

    void print(map::Obstacle obstacle);

    void print(map::Map& map);

    void print(map::OccupancyGrid& grid);

    void print(map::OccupancyMap& occupancy_map);

    void print(units::Radian angle);

    void print(vehicle::Pose pose);

    void print(vehicle::DriveState state);

    void print(planner::NodeState state); 


}


#endif 
