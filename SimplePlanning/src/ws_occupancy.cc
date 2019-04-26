#include <iostream> 
#include <assert.h> 

#include "Map.h"
#include "Geometry.h" 
#include "VehicleModel.h" 
#include "Display.h"
#include "MetricMath.h" 




map::Obstacle createObstacle(geometry::Point point, geometry::Dimension dimension){
    geometry::RectangleGeometry rect1(point,dimension); 
    map::Obstacle wall1(rect1); 
    return wall1; 
}

map::StaticObstaclesMap createTestStaticMap(){
    geometry::Point p1, p2; 
    geometry::Dimension dimension1, dimension2, map_dimension; 
    //
    p1.x = units::Meter{2.5}; p1.y = units::Meter{3.5};
    dimension1.width = units::Meter{3}; 
    dimension1.height = units::Meter{1.5};
    //
    p2.x = units::Meter{7.5}, p2.y = units::Meter{3.5}; 
    dimension2.width = units::Meter{2};
    dimension2.height = units::Meter{2.5}; 
    // map dimensions 
    map_dimension.width = units::Meter{10};
    map_dimension.height = units::Meter{5}; 

    map::Obstacle wall1 = createObstacle(p1,dimension1);
    map::Obstacle wall2 = createObstacle(p2,dimension2);                                      
    // Create Map                                       
    map::StaticObstaclesMap map1 (map_dimension);         
    map1.addObstacle(wall1); 
    map1.addObstacle(wall2); 
    return map1;     
}

void testObstacle(){
    geometry::Point p1,p2,p3; 
    geometry::Dimension dim1, dim2, dim3; 
    p1.x = units::Meter{0.5}; 
    p1.y = units::Meter{0.5}; 
    dim1.width = units::Meter{0.25};
    dim1.height = units::Meter{0.25}; 
    //
    p2.x = units::Meter{0};
    p2.y = units::Meter{1.0}; 
    dim2.width = units::Meter{0.25}; 
    dim2.height = units::Meter{0.25}; 
    // 
    p3.x = units::Meter{0.0}; 
    p3.y = units::Meter{0.5}; 
    dim3.width  = units::Meter{0.25}; 
    dim3.height = units::Meter{0.25}; 

    map::Obstacle wall1 = createObstacle(p1,dim1); 

    map::Obstacle wall2 = createObstacle(p2,dim2); 

    map::Obstacle wall3 = createObstacle(p3,dim3);                                                                             
                         
    std::cout << "wall 1 ";
    display::print(wall1);                                 
}

void testMap(){
    // Create Obstacles 
    map::StaticObstaclesMap map1  = createTestStaticMap(); 
    // Get and display Obstacles. 
    display::print(map1);
}

void testOcuppancyMap(){
    map::StaticObstaclesMap map1 = createTestStaticMap(); 
    map::OccupancyGrid grid(units::Meter{0.1},map1.getDimension()); 
    map::OccupancyMap occupancy_map(grid); 
    occupancy_map.setMap(map1); 
    occupancy_map.computeOccupancy(); 

    display::print(map1);
    display::print(occupancy_map); 
}



 
int main(){

 testOcuppancyMap(); 


}