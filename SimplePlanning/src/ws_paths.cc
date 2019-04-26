#include <iostream> 
#include <assert.h> 

#include "Map.h"
#include "Geometry.h" 
#include "VehicleModel.h" 
#include "Display.h"
#include "MetricMath.h" 
#include "PlannerVariant.h" 

const std::string GENERATED_PLAN_DIR = "./generated_paths/";

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

map::StaticObstaclesMap createTestStaticMap2(){
    geometry::Point p1, p2, p3,p4; 
    geometry::Dimension dimension1, dimension2,dimension3,dimension4, map_dimension; 
    // top obstacle 
    p1.x = units::Meter{3.5}; p1.y = units::Meter{10};
    dimension1.width = units::Meter{2.5}; 
    dimension1.height = units::Meter{1.5};
    // middle obstacle 
    p2.x = units::Meter{3.5}, p2.y = units::Meter{6.5}; 
    dimension2.width = units::Meter{6.5};
    dimension2.height = units::Meter{2.5}; 
    // side obstacles 
    p3.x = units::Meter{0}, p3.y = units::Meter{4.5}; 
    dimension3.width = units::Meter{0.8};
    dimension3.height = units::Meter{1.5}; 

    p4.x = units::Meter{0}, p4.y = units::Meter{2.2}; 
    dimension4.width = units::Meter{0.8};
    dimension4.height = units::Meter{1.5}; 
    // map dimensions 
    map_dimension.width = units::Meter{10};
    map_dimension.height = units::Meter{10}; 

    map::Obstacle wall1 = createObstacle(p1,dimension1);
    map::Obstacle wall2 = createObstacle(p2,dimension2); 
    map::Obstacle wall3 = createObstacle(p3,dimension3); 
    map::Obstacle wall4 = createObstacle(p4,dimension4);                                      
    // Create Map                                       
    map::StaticObstaclesMap map1 (map_dimension);         
    map1.addObstacle(wall1); 
    map1.addObstacle(wall2); 
    map1.addObstacle(wall3); 
    map1.addObstacle(wall4); 
    return map1;     
}


map::OccupancyMap createPlannerOccupancyMap(map::StaticObstaclesMap& map1){    
    map::OccupancyGrid grid(units::Meter{0.1},map1.getDimension()); 
    map::OccupancyMap occupancy_map(grid); 
    occupancy_map.setMap(map1); 
    occupancy_map.computeOccupancy(); 
    return occupancy_map; 
}



void hybridAstarPlanner(vehicle::DriveState start_drivestate,
                                vehicle::DriveState goal_drivestate,
                                const vehicle::SimpleCar& vehicleModel,
                                const map::StaticObstaclesMap& static_map,
                                const map::OccupancyMap& occupancy_map){
    std::cout << "hybridAstarPlanner " << std::endl; 
    // create a start drive state. 
    units::Radian orientation_resolution = metricmath::degreeToRadian(units::Degree{2});
    units::Radian steering_resolution    = metricmath::degreeToRadian(units::Degree{10});
    planner::HybridAstarPlanner pathplanner(occupancy_map,orientation_resolution,
                                                          steering_resolution);    
    pathplanner.setVehicleModel(vehicleModel);
    std::cout <<"Making Plan " << std::endl; 
    // make a plan 
    pathplanner.makeplan(start_drivestate, goal_drivestate); 
    // retrieve the plan 
    // write plan to file.  
    planner::writePlanToFile(GENERATED_PLAN_DIR + "hybrid.tplan",
                                pathplanner,static_map,vehicleModel); 
    // Note: this depends on the map. 
    std::cout <<"hybridAstarPlanner done " << std::endl;   
    // first planner  

}

void hybridAstarPlannerWithPressure(vehicle::DriveState start_drivestate,
                                vehicle::DriveState goal_drivestate,
                                const vehicle::SimpleCar& vehicleModel,
                                const map::StaticObstaclesMap& static_map,
                                const map::OccupancyMap& occupancy_map){
    std::cout << "hybridAstarPlanner " << std::endl; 
    // create a start drive state. 
    units::Radian orientation_resolution = metricmath::degreeToRadian(units::Degree{2});
    units::Radian steering_resolution    = metricmath::degreeToRadian(units::Degree{10});
    planner::HybridAstarWithObstaclePressure pathplanner(occupancy_map,orientation_resolution,
                                                          steering_resolution);    
    pathplanner.setVehicleModel(vehicleModel);
    std::cout <<"Making Plan " << std::endl; 
    // make a plan 
    pathplanner.makeplan(start_drivestate, goal_drivestate); 
    // write plan to file.  
    planner::writePlanToFile(GENERATED_PLAN_DIR + "hybridpressure.tplan",
                                pathplanner,static_map,vehicleModel); 
    // Note: this depends on the map. 
    std::cout <<"hybridAstarPlannerWithPressure passed " << std::endl;   
    // first planner  

}

void testDifferentPlanner(){
            // create map 
    map::StaticObstaclesMap map1 = createTestStaticMap();    
    map::OccupancyMap occupancy_map = createPlannerOccupancyMap(map1);
    display::print(map1);
    display::print(occupancy_map);     
    // create the vehicle model 
    vehicle::SimpleCarSteeringConfig steeringConfig;
    steeringConfig.maxSteering = units::Radian{15 * 3.141 / 180.0};
    steeringConfig.instantSteering = units::Radian{15 * 3.141 / 180.0};
    vehicle::SimpleCarDimension carDimension; 
    carDimension.length = units::Meter{0.2}; 
    carDimension.width  = units::Meter{0.1}; 
    vehicle::SimpleCar vehicleModel(carDimension,steeringConfig);  
 
    vehicle::DriveState start_drivestate; 
    start_drivestate.pose.x = units::Meter{4};
    start_drivestate.pose.y = units::Meter{1.5}; 
    start_drivestate.pose.orientation = metricmath::degreeToRadian(units::Degree{0}); 
    start_drivestate.steeringAngle = units::Radian{0}; 
    // create goal drive state 
    vehicle::DriveState goal_drivestate; 
    goal_drivestate.pose.x = units::Meter{4};
    goal_drivestate.pose.y = units::Meter{4.5}; 
    goal_drivestate.pose.orientation = metricmath::degreeToRadian(units::Degree{0}); 
    goal_drivestate.steeringAngle = units::Radian{0}; 

    // Test the different planners. 
    hybridAstarPlanner(start_drivestate,goal_drivestate,vehicleModel,map1,occupancy_map); 
    std::cout <<" ###############################################################################"<<std::endl;
    hybridAstarPlannerWithPressure(start_drivestate,goal_drivestate,vehicleModel,map1,occupancy_map); 
}


void testDifferentPlanner2(){
            // create map 
    map::StaticObstaclesMap map1 = createTestStaticMap2();    
    map::OccupancyMap occupancy_map = createPlannerOccupancyMap(map1);
    display::print(map1);
    display::print(occupancy_map);     
    // create the vehicle model 
    vehicle::SimpleCarSteeringConfig steeringConfig;
    steeringConfig.maxSteering = units::Radian{15 * 3.141 / 180.0};
    steeringConfig.instantSteering = units::Radian{15 * 3.141 / 180.0};
    vehicle::SimpleCarDimension carDimension; 
    carDimension.length = units::Meter{0.2}; 
    carDimension.width  = units::Meter{0.1}; 
    vehicle::SimpleCar vehicleModel(carDimension,steeringConfig);  
 
    vehicle::DriveState start_drivestate; 
    start_drivestate.pose.x = units::Meter{9};
    start_drivestate.pose.y = units::Meter{1}; 
    start_drivestate.pose.orientation = metricmath::degreeToRadian(units::Degree{180}); 
    start_drivestate.steeringAngle = units::Radian{0}; 
    // create goal drive state 
    vehicle::DriveState goal_drivestate; 
    goal_drivestate.pose.x = units::Meter{8};
    goal_drivestate.pose.y = units::Meter{9}; 
    goal_drivestate.pose.orientation = metricmath::degreeToRadian(units::Degree{270}); 
    goal_drivestate.steeringAngle = units::Radian{0}; 

    // Test the different planners. 
    hybridAstarPlanner(start_drivestate,goal_drivestate,vehicleModel,map1,occupancy_map); 
    std::cout <<" ###############################################################################"<<std::endl;
    hybridAstarPlannerWithPressure(start_drivestate,goal_drivestate,vehicleModel,map1,occupancy_map); 
}



 
int main(){
    // testDifferentPlanner();
    testDifferentPlanner2();

}