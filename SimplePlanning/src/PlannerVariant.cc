#include "PlannerVariant.h" 


namespace planner{

/**
 *  HybridAstarWithObstaclePressure
 * */
 HybridAstarWithObstaclePressure::HybridAstarWithObstaclePressure(const map::OccupancyMap& occupancy_map,
                        units::Radian orientation_resolution,
                        units::Radian steering_resolution):
HybridAstarPlanner(occupancy_map,orientation_resolution,steering_resolution){
     _name = "HybridAstarWithObstaclePressure";
}

// computeProximityCost
double HybridAstarWithObstaclePressure::computeProximityCost(vehicle::Pose pose)const{
    geometry::Point point; 
    point.x = pose.x; 
    point.y = pose.y; 
    map::GridPoint gridpoint = _occupancyMap->convertToGridPoint(point); 
    // get all the neighboring grids in a 4x4 square. 
    double cost = 0;
    int nleft_right = 2; 
    int ntop_bottom = nleft_right;
    map::GridPoint gp; 
    for (int row = gridpoint.row - ntop_bottom; row <= gridpoint.row + ntop_bottom; row++){
        for (int col = gridpoint.column - nleft_right; col <= gridpoint.column + nleft_right ; col++){
            gp.row = row; gp.column = col; 
            if ((row < 0) || (col < 0) || !_occupancyMap->isFree(gp) ){
                double dist = std::abs(gridpoint.row - row) + std::abs(gridpoint.column - col);
                // double dist = std::sqrt( std::pow(gridpoint.row - row, 2) + std::pow(gridpoint.column - col,2));
                cost += (1/dist); 
                
            }
        }
    }
    
    return cost; 
}

//
double HybridAstarWithObstaclePressure::computeHeuristicCostOfStateNode(NodeState state) const{  
    // (i) use euclidean distance. 
    NodeState goal_state = _goalNodeState;   
    units::Meter distance = computeEuclideanDistanceFromPose(state.info.driveState.pose,
                                              goal_state.info.driveState.pose); 
    // (ii) use an obstacle around. 
    double proximity_cost = computeProximityCost(state.info.driveState.pose); 
    double total = distance.value * 0.5 + proximity_cost * 10 ; 
    // std::cout << " cost = "<<total<< std::endl; 
    return total; 
}

// getHash 
std::string HybridAstarWithObstaclePressure::getHash(NodeState state) const{
    std::string hash = "" + std::to_string(state.xId) + "|"
                          + std::to_string(state.yId) + "|"
                          + std::to_string(state.orientationId) + "|"
                          + std::to_string(state.directionId); 
    return hash; 
}
//
double HybridAstarWithObstaclePressure::computeCostOfReachingStateNode(NodeState from_node , NodeState to_node, PlanningCommand command) const{
    double cost = HybridAstarPlanner::computeCostOfReachingStateNode(from_node,to_node,command);
    units::Degree prevdeg = metricmath::radianToDegree(from_node.info.driveState.steeringAngle); 
    units::Degree deg = metricmath::radianToDegree(to_node.info.driveState.steeringAngle); 

    cost = cost + abs(deg.value - prevdeg.value) * 2; 
    if ( command.speed.value < 0){
        cost += 5; 
    }
    return cost ; 

} 

//
bool HybridAstarWithObstaclePressure:: isGoalState(NodeState state, NodeState goal) const{
    bool result = ( state.xId == goal.xId && 
                    state.yId == goal.yId && 
                    abs(state.orientationId - goal.orientationId) <= 1 );
    return result;
    // return HybridAstarPlanner::isGoalState(state,goal);
}

//getReachableNodeStates
// TODO: for now, reset the steeringAngle to zero. 
// and do not include it in the planning dimensions. 
std::vector<NodeState> HybridAstarWithObstaclePressure::getReachableNodeStates(NodeState parent_state) const{
    // retrieve the drive state, and reset the steering angle to zero. 
    vehicle::DriveState drivestate = parent_state.info.driveState; 
    drivestate.steeringAngle = metricmath::degreeToRadian(units::Degree{0});
    // create planning command 
    units::Meter gridres = _occupancyMap->getResolution(); 
    double speed_value = std::sqrt(2 * gridres.value * gridres.value) * 1.5; 
    //
    PlanningCommand command; 
    command.timeStep = units::Second {1}; 
    command.speed    = units::MeterPerSecond{speed_value}; 
    command.steeringAngle = units::Radian{0};   
    command.steeringAngle = units::Radian{0};   
    // with steering angles: 10 deg(left), 0deg, 10 deg(right) 
    std::vector<units::Radian> steer_angles;
    // {-5,-1,0,1,5}
    for ( double x : {-40,0,40}){
        steer_angles.push_back( metricmath::degreeToRadian(units::Degree{x}));        
    }
    std::vector<units::MeterPerSecond> speeds = {units::MeterPerSecond{0.1},
                                                units::MeterPerSecond{-0.1}}; 
    // get the hash of this state
    std::string parentStateHash = getHash(parent_state); 
    // forward at 0deg steering 
    std::vector<NodeState> result_states;
    double cost = 0; 
    for ( units::Radian steer_angle : steer_angles){  
        command.steeringAngle = steer_angle; 
        for (units::MeterPerSecond speed : speeds ){
            command.speed = speed; 
            NodeState node_state = getNextStateAfterMove(drivestate,command);
            
            // verify that this new state is valid. 
            cost = 0; 
            if ( !isProcessed(node_state) && verifyNodeState(node_state) ){
                // compute the cost of getting to this node from the start. 
                cost = parent_state.info.totalCost; 
                cost += computeCostOfReachingStateNode(parent_state,node_state,command); 
                cost += computeHeuristicCostOfStateNode(node_state);
                //
                node_state.info.totalCost = cost;
                node_state.parentStateHash = parentStateHash; // reference to parent...
                //          
                result_states.push_back(node_state);
            }
        }

    }    
    return result_states; 
}


}// namespace planner 