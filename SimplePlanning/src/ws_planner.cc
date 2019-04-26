#include <iostream> 
#include <assert.h> 

#include "Map.h"
#include "Geometry.h" 
#include "VehicleModel.h" 
#include "Display.h"
#include "MetricMath.h" 
#include "PlannerVariant.h" 



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


map::OccupancyMap createPlannerOccupancyMap(map::StaticObstaclesMap& map1){    
    map::OccupancyGrid grid(units::Meter{0.1},map1.getDimension()); 
    map::OccupancyMap occupancy_map(grid); 
    occupancy_map.setMap(map1); 
    occupancy_map.computeOccupancy(); 
    return occupancy_map; 
}


void testPlanner_isGoalState(){
    std::cout << "testPlanner_isGoalState " << std::endl; 
    // create map 
    map::StaticObstaclesMap map1 = createTestStaticMap();    
    map::OccupancyMap occupancy_map = createPlannerOccupancyMap(map1);
    display::print(map1);
    display::print(occupancy_map); 
    // create planner 
    units::Radian orientation_resolution = metricmath::degreeToRadian(units::Degree{20});
    units::Radian steering_resolution    = metricmath::degreeToRadian(units::Degree{10});

    planner::HybridAstarPlanner pathplanner(occupancy_map,orientation_resolution,
                                                          steering_resolution);
    
    display::print(pathplanner.getOrientationResolution()); 
    display::print(pathplanner.getSteeringResolution()); 
    // create first state 
    planner::NodeState state1; 
    state1.xId = 1; 
    state1.yId = 5; 
    state1.orientationId = 3; 
    state1.steeringId = 2; 
    //
    planner::NodeState state2; 
    state2.xId = 2; 
    state2.yId = 4; 
    state2.orientationId = 5; 
    state2.steeringId = 20; 
    // create second state 
    planner::NodeState goal_state; 
    goal_state.xId = 2; 
    goal_state.yId = 4;
    goal_state.orientationId = 5; 
    goal_state.steeringId = 9; 

    bool result1 = pathplanner.isGoalState(state1,goal_state); 
    bool result2 = pathplanner.isGoalState(state2,goal_state); 

    assert(result1 == false); 
    assert(result2 == true ); 
}

void testPlanner_getHash(){
    std::cout << "testPlanner_getHash " << std::endl; 
    // create map 
    map::StaticObstaclesMap map1 = createTestStaticMap();    
    map::OccupancyMap occupancy_map = createPlannerOccupancyMap(map1);
    display::print(map1);
    display::print(occupancy_map); 
    // create planner 
    units::Radian orientation_resolution = metricmath::degreeToRadian(units::Degree{20});
    units::Radian steering_resolution    = metricmath::degreeToRadian(units::Degree{10});

    planner::HybridAstarPlanner pathplanner(occupancy_map,orientation_resolution,
                                                          steering_resolution);

    planner::NodeState goal_state; 
    goal_state.xId = 2; 
    goal_state.yId = 4;
    goal_state.orientationId = 5; 
    goal_state.steeringId = 9; 

    std::string true_hash = "2|4|5"; 
    std::string hash = pathplanner.getHash(goal_state); 
    assert(hash == true_hash); 
    std::cout << "testPlanner_getHash passed"<< std::endl; 

}

void testPlanner_verifyNodeState(){
    std::cout << "testPlanner_verifyNodeState " << std::endl; 
    // create map 
    map::StaticObstaclesMap map1 = createTestStaticMap();    
    map::OccupancyMap occupancy_map = createPlannerOccupancyMap(map1);
    display::print(map1);
    display::print(occupancy_map); 
    // create planner 
    units::Radian orientation_resolution = metricmath::degreeToRadian(units::Degree{20});
    units::Radian steering_resolution    = metricmath::degreeToRadian(units::Degree{10});

    planner::HybridAstarPlanner pathplanner(occupancy_map,orientation_resolution,
                                                          steering_resolution);
    // create a valid state in the map 
    planner::NodeState stateInMap; 
    stateInMap.xId = 2; 
    stateInMap.yId = 4;
    stateInMap.orientationId = 5; 
    stateInMap.steeringId = 9; 
    // create an invalid state in an obstacle   
    planner::NodeState stateInObstacle; 
    stateInObstacle.xId = 40;
    stateInObstacle.yId = 25;
    stateInObstacle.orientationId = 3;
    stateInObstacle.steeringId = 0; 
    // create an invalid state outside grid 
    planner::NodeState stateOutsideMap; 
    stateOutsideMap.xId = 102; 
    stateOutsideMap.yId = 49; 
    stateOutsideMap.orientationId = 1000; 
    
    assert(pathplanner.verifyNodeState(stateInObstacle)== false); 
    assert(pathplanner.verifyNodeState(stateOutsideMap) == false); 
    assert(pathplanner.verifyNodeState(stateInMap) == true); 

    std::cout << "testPlanner_verifyNodeState passed"<< std::endl; 
}

void testPlanner_getNodeState(){
    std::cout << "testPlanner_getNodeState " << std::endl;         
    // create map 
    map::StaticObstaclesMap map1 = createTestStaticMap();    
    map::OccupancyMap occupancy_map = createPlannerOccupancyMap(map1);
    display::print(map1);
    display::print(occupancy_map);     
    // create the vehicle model 
    vehicle::SimpleCarSteeringConfig steeringConfig;
    steeringConfig.maxSteering = units::Radian{15 * 3.141 / 180.0};
    steeringConfig.instantSteering = units::Radian{15 * 3.141 / 180.0};

    vehicle::SimpleCarDimension dimension; 
    dimension.length = units::Meter{0.2}; 
    vehicle::SimpleCar vehicleModel(dimension,steeringConfig);     
    // create planner 
    units::Radian orientation_resolution = metricmath::degreeToRadian(units::Degree{20});
    units::Radian steering_resolution    = metricmath::degreeToRadian(units::Degree{10});

    planner::HybridAstarPlanner pathplanner(occupancy_map,orientation_resolution,
                                                          steering_resolution);    
    pathplanner.setVehicleModel(vehicleModel); 
    // create a drive state 
    vehicle::DriveState drivestate; 
    drivestate.pose.x = units::Meter{4};
    drivestate.pose.y = units::Meter{2.5}; 
    drivestate.pose.orientation = metricmath::degreeToRadian(units::Degree{45}); 
    drivestate.steeringAngle = units::Radian{0}; 
    // 
    planner::NodeState nodestate = pathplanner.getNodeState(drivestate); 
    display::print(nodestate); 
    // assert that the node states ids are 40, 25, 2. 
    // Note: this depends on the map. 
    assert(nodestate.xId == 40); 
    assert(nodestate.yId == 25); 
    assert(nodestate.orientationId == 2); 
    std::cout <<"testPlanner_getNodeState passed " << std::endl; 
}

void testPlanner_getReachableNodeStates(){
    std::cout << "testPlanner_getReachableNodeStates " << std::endl; 
    // create map 
    map::StaticObstaclesMap map1 = createTestStaticMap();    
    map::OccupancyMap occupancy_map = createPlannerOccupancyMap(map1);
    display::print(map1);
    display::print(occupancy_map);     
    // create the vehicle model 
    vehicle::SimpleCarSteeringConfig steeringConfig;
    steeringConfig.maxSteering = units::Radian{15 * 3.141 / 180.0};
    steeringConfig.instantSteering = units::Radian{15 * 3.141 / 180.0};
    
    vehicle::SimpleCarDimension dimension; 
    dimension.length = units::Meter{0.2}; 
    vehicle::SimpleCar vehicleModel(dimension,steeringConfig);     
    // create planner 
    units::Radian orientation_resolution = metricmath::degreeToRadian(units::Degree{20});
    units::Radian steering_resolution    = metricmath::degreeToRadian(units::Degree{10});
    planner::HybridAstarPlanner pathplanner(occupancy_map,orientation_resolution,
                                                          steering_resolution);    
    pathplanner.setVehicleModel(vehicleModel); 
    // create a drive state 
    vehicle::DriveState drivestate; 
    drivestate.pose.x = units::Meter{4};
    drivestate.pose.y = units::Meter{1.5}; 
    drivestate.pose.orientation = metricmath::degreeToRadian(units::Degree{0}); 
    drivestate.steeringAngle = units::Radian{0}; 
    // get nodestate from planner 
    planner::NodeState nodestate = pathplanner.getNodeState(drivestate);     
    std::cout <<"Current Node state " << std::endl; 
    display::print(nodestate);     
    display::print(drivestate); 

    // get all reachable states 
    std::vector<planner::NodeState> reachable = pathplanner.getReachableNodeStates(nodestate); 
    // assert that the node states ids are 40, 25, 2. 
    std::cout << "reachable = " << reachable.size() << std::endl; 
    for (planner::NodeState nodestate : reachable){
        std::cout <<"#####################"<< std::endl;
        display::print(nodestate); 
        display::print(nodestate.info.driveState); 
    }
    // Note: this depends on the map. 
    std::cout <<"testPlanner_getReachableNodeStates passed " << std::endl; 
}

void testPlanner_SimplePlan(){

    std::cout << "testPlanner_SimplePlan " << std::endl; 
    // create map 
    map::StaticObstaclesMap map1 = createTestStaticMap();    
    map::OccupancyMap occupancy_map = createPlannerOccupancyMap(map1);
    display::print(map1);
    display::print(occupancy_map);     
    // create the vehicle model 
    vehicle::SimpleCarSteeringConfig steeringConfig;
    steeringConfig.maxSteering = units::Radian{15 * 3.141 / 180.0};
    steeringConfig.instantSteering = units::Radian{15 * 3.141 / 180.0};

    vehicle::SimpleCarDimension dimension; 
    dimension.length = units::Meter{0.2}; 
    vehicle::SimpleCar vehicleModel(dimension,steeringConfig);     
    // create planner 
    units::Radian orientation_resolution = metricmath::degreeToRadian(units::Degree{2});
    units::Radian steering_resolution    = metricmath::degreeToRadian(units::Degree{10});
    planner::HybridAstarPlanner pathplanner(occupancy_map,orientation_resolution,
                                                          steering_resolution);    
    pathplanner.setVehicleModel(vehicleModel); 
    
    // create a start drive state. 
    vehicle::DriveState start_drivestate; 
    start_drivestate.pose.x = units::Meter{4};
    start_drivestate.pose.y = units::Meter{1.5}; 
    start_drivestate.pose.orientation = metricmath::degreeToRadian(units::Degree{0}); 
    start_drivestate.steeringAngle = units::Radian{0}; 
    // create goal drive state 
    vehicle::DriveState goal_drivestate; 
    goal_drivestate.pose.x = units::Meter{9};
    goal_drivestate.pose.y = units::Meter{4}; 
    goal_drivestate.pose.orientation = metricmath::degreeToRadian(units::Degree{0}); 
    goal_drivestate.steeringAngle = units::Radian{0}; 
    
    std::cout <<"Making Plan " << std::endl; 
    // make a plan 
    pathplanner.makeplan(start_drivestate, goal_drivestate); 
    // retrieve the plan 
    std::vector<vehicle::DriveState> plan = pathplanner.getPlan(); 
    std::ofstream outputfile; 
    outputfile.open("./generated_paths/simpleplan.tplan");
    planner::writePlannerToOutput(outputfile,pathplanner); 
    outputfile.close();
    // Note: this depends on the map. 
    std::cout <<"testPlanner_SimplePlan passed " << std::endl;     
}

 
int main(){
 
    testPlanner_SimplePlan();

}