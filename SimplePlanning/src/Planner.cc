#include "Planner.h" 

namespace planner{

//LessThanByDistanceFromStart 
bool GreaterThanByTotalCostFromStart::operator()(const NodeState& lhs, const NodeState& rhs ) const{
    return lhs.info.totalCost > rhs.info.totalCost; 
}


/**
 * HybridAstarPlanner
 * DriveState is 4D : x, y, orientation, forward/reverse. 
*/
// HybridAstarPlanner 
HybridAstarPlanner::HybridAstarPlanner(const map::OccupancyMap& occupancy_map,units::Radian orientation_resolution,
                                     units::Radian steering_resolution):
_orientationResolution(orientation_resolution),
_steeringResolution(steering_resolution){
    _name = "HybridAstarPlanner";
    _occupancyMap = &occupancy_map; 
}
// makeplan
void HybridAstarPlanner::makeplan(vehicle::DriveState start, vehicle::DriveState goal){ 
    // initialiseQueue
    initializeQueue(); 
    // get the NodeState representation of start and goal 
    NodeState startNodeState = getNodeState(start); 
    NodeState goalNodeState  = getNodeState(goal); 
    startNodeState.info.totalCost = 0; 
    setStartNodeState(startNodeState); 
    setGoalNodeState(goalNodeState); 
    // add to queue.  
    pushIntoQueue(startNodeState);
    //
    NodeState currentNodeState = startNodeState;      
    bool is_goal_reached = false; 
    while ( !isQueueEmpty() ){
        currentNodeState = popFromQueueTop(); 
        // check if current-node is the goal-node. 
        if ( isGoalState(currentNodeState,goalNodeState) ){
            is_goal_reached = true;             
            break; 
        }
        // check if the node is processed already. 
        if ( !isProcessed(currentNodeState) ){            
            markAsProcessed(currentNodeState);             
            // get all reacheable states from the current state. 
            // reacheable states are those gotten to from current-state after executing 
            // each of the possible state-transition functions. 
            std::vector<NodeState> reacheableNodeStates = getReachableNodeStates(currentNodeState);             
            for ( NodeState& state : reacheableNodeStates ){                   
                // add to queue
                pushIntoQueue(state); 
            }
        }
    }
    if (is_goal_reached){
        buildPath(currentNodeState); 
        processGoalState(currentNodeState); 
    }
                                    
}

// setStartNodeState
void HybridAstarPlanner::setStartNodeState(NodeState start_node_state){
    _startNodeState = start_node_state; 
}
// setGoalNodeState 
void HybridAstarPlanner::setGoalNodeState(NodeState goal_node_state){
    _goalNodeState = goal_node_state; 
}

// getPath 
std::vector<vehicle::DriveState> HybridAstarPlanner::getPlan() const {
    return _path; 
}

// computeCostOfReachingStateNode
// this computes the cost of reaching a node from a given node. 
// TODO: ideally, this cost should be computed when calculating the 
// reachables nodes. for now, this would do. 
double HybridAstarPlanner::computeCostOfReachingStateNode(NodeState from_node, NodeState to_node,
                            PlanningCommand command) const { 
    // TODO: let this cost incorporate. 
    // cost of making the steering turn, 
    // cost of distance. 
    // cost of proximity to obstacle.
    units::Degree deg = metricmath::radianToDegree(command.steeringAngle);
    double value = 1;// std::abs(deg.value) ;  
    // std::cout << value << std::endl; 
    return value; 
}
// computeEuclideanDistanceFromPose
units::Meter HybridAstarPlanner::computeEuclideanDistanceFromPose(vehicle::Pose pose1, vehicle::Pose pose2) const {    
    double dist = std::sqrt( std::pow(pose1.x.value - pose2.x.value,2) + std::pow(pose1.y.value - pose2.y.value,2) );
    // std::cout << dist << std::endl; 
    return units::Meter{dist}; 
}
// computeHeuristicCostOfStateNode
// this returns any heuristic cost associated with 
// a state. Here we use euclideanDistance
double HybridAstarPlanner::computeHeuristicCostOfStateNode(NodeState state) const{  
    NodeState goal_state = _goalNodeState;   
    units::Meter distance = computeEuclideanDistanceFromPose(state.info.driveState.pose,
                                              goal_state.info.driveState.pose); 
    
    return distance.value; 
}

// isGoalState 
// Returns True if the state is at the goal state for (xId, yId & orientationId ),
// and false otherwise. Note: steeringId is not used. 
// any steering that results in reaching the goal state is Acceptable. 
bool HybridAstarPlanner::isGoalState(NodeState state, NodeState current_node_state) const{
    vehicle::DriveState drivestate1 = state.info.driveState; 
    vehicle::DriveState drivestate2 = current_node_state.info.driveState; 
    units::Meter resolution = _occupancyMap->getResolution(); 
    units::Meter distance = computeEuclideanDistanceFromPose(drivestate1.pose,drivestate2.pose); 
    return (distance.value <= resolution.value); 

    // bool result = ( state.xId == current_node_state.xId && 
    //                 state.yId == current_node_state.yId && 
    //                 state.orientationId == current_node_state.orientationId );
    // return result;                     
}

// isProcessed 
// returns true if this node-state has been processed 
bool HybridAstarPlanner::isProcessed(NodeState state) const { 
    std::string hash = getHash(state); 
    auto search = _nodeInfo.find(hash); 
    return ( search == _nodeInfo.end() )? false : true; 
}

//popFromQueueTop
// returns the node state at the top of the priority-queue 
NodeState HybridAstarPlanner::popFromQueueTop(){
    NodeState state = _priorityQueue.top(); 
    _priorityQueue.pop(); 
    return state; 
}

//markAsProcessed
void HybridAstarPlanner::markAsProcessed(NodeState state){
    std::string hash = getHash(state); 
    // insert state into 
    _nodeInfo[hash] = state;    
}

//pushIntoQueue 
void HybridAstarPlanner::pushIntoQueue(NodeState state){    
    // add to queue. 
    _priorityQueue.push(state); 
}

// isPathFound() 
bool HybridAstarPlanner::isPathFound() const{
    return _path.size() > 0; 
}

// printPath 
void HybridAstarPlanner::buildPath(NodeState state) {
    _path = std::vector<vehicle::DriveState>(); 
    NodeState current_state = state; 
    while ( current_state.parentStateHash != EMPTY_HASH){        
        _path.push_back(current_state.info.driveState); 
        current_state = lookUpNodeStateByHash(current_state.parentStateHash); 
    }
    std::reverse(_path.begin(),_path.end()); 
}

// lookUPNodeStateByHash 
NodeState HybridAstarPlanner::lookUpNodeStateByHash(std::string state_hash) const {
    return _nodeInfo.at(state_hash); 
}


// processGoalState 
void HybridAstarPlanner::processGoalState(NodeState state) const{
    std::cout <<"####################################################"<<std::endl; 
    std::cout << "Goal State Reached !!! "<< std::endl;
    std::cout << " xId = "<< state.xId
              << " yId = "<< state.yId<< std::endl;
    std::cout << "Goal State reached " << std::endl; 
    std::cout << "Path = " << std::endl; 
    // 
    std::stack<NodeState> tmp; 
    NodeState current_state = state; 
    while ( current_state.parentStateHash != EMPTY_HASH){        
        tmp.push(current_state); 
        current_state = lookUpNodeStateByHash(current_state.parentStateHash); 
    }
    // display all. 
    while ( !tmp.empty() ){
        current_state = tmp.top(); 
        tmp.pop(); 
        showStateOnScreen(current_state); 
    }

}

// initializeQueue
// initializes the empty queue
void HybridAstarPlanner::initializeQueue(){
    // initialise path 
    _path = std::vector<vehicle::DriveState>(); 
    // initialise nodeinfo. 
    _nodeInfo = std::map<std::string, NodeState>();
    // initialise priority queue 
    _priorityQueue = std::priority_queue<NodeState,std::vector<NodeState>,GreaterThanByTotalCostFromStart>();
}

//isQueueEmpty
bool HybridAstarPlanner::isQueueEmpty() const{
    return _priorityQueue.empty();
}

// getNodeState
// returns the node state corresponding to drive state 
NodeState HybridAstarPlanner::getNodeState(vehicle::DriveState drivestate) const { 
    geometry::Point point; 
    point.x = drivestate.pose.x; 
    point.y = drivestate.pose.y; 

    map::GridPoint gridpoint = _occupancyMap->convertToGridPoint(point);
    //
    NodeState node; 
    node.xId = gridpoint.column; 
    node.yId = gridpoint.row; 
    node.orientationId = getOrientationId(drivestate.pose.orientation); 
    node.steeringId    = getSteeringId(drivestate.steeringAngle);
    NodeStateInfo node_info; 
    node_info.driveState = drivestate; 
    node.info = node_info;
    // TODO: When do we update the distanceFromStart and totalDistance ? 
    return node; 
}

// getNodeStateWithInfo 
// NodeStateWithInfo HybridAstarPlanner::getNodeStateWithInfo(vehicle::DriveState drivestate) const{
//     NodeState state = getNodeState(drivestate); 
//     NodeStateWithInfo state_info; 
//     state_info.state = state; 
//     state_info.  = NodeInfo{drivestate}; 
//     return state_info; 
// }

//getSteeringId
int HybridAstarPlanner::getSteeringId(units::Radian steering_angle) const{           
    int loc = (steering_angle.value) / _steeringResolution.value; 
    if ( steering_angle.value < 0){
        units::Radian max_steering_angle = _carModel->getMaxSteering(); 
        int nsize = (int)(max_steering_angle.value / _steeringResolution.value) + 1; 
        loc = loc + nsize; 
    }
    return loc ; 
}

//getOrientationId
int HybridAstarPlanner::getOrientationId(units::Radian orientation) const{
    int loc = (orientation.value ) / _orientationResolution.value; 
    return loc; 
}

//getHash
std::string HybridAstarPlanner::getHash(NodeState state) const{
    std::string hash = "" + std::to_string(state.xId) + "|"
                          + std::to_string(state.yId) + "|"
                          + std::to_string(state.orientationId); 
    return hash; 
}

// getNextStateAfterMove
NodeState HybridAstarPlanner::getNextStateAfterMove(vehicle::DriveState drivestate, PlanningCommand command )const {
    vehicle::DriveState new_drive_state = _carModel->getNextStateAfterMove(drivestate,command.speed,
                                                                                    command.steeringAngle,
                                                                                    command.timeStep);     
    NodeState node_state = getNodeState(new_drive_state);    
    node_state.directionId = (command.speed.value  > 0)? 1 : 0;  

    return node_state;    
}

//getReachableNodeStates
// TODO: for now, reset the steeringAngle to zero. 
// and do not include it in the planning dimensions. 
std::vector<NodeState> HybridAstarPlanner::getReachableNodeStates(NodeState parent_state) const{
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
    // with steering angles: 10 deg(left), 0deg, 10 deg(right) 
    std::vector<units::Radian> steer_angles;
    for ( double x : {-40,0,40}){
        steer_angles.push_back( metricmath::degreeToRadian(units::Degree{x}));        
    }
    // get the hash of this state
    std::string parentStateHash = getHash(parent_state); 
    // forward at 0deg steering 
    std::vector<NodeState> result_states;
    double cost = 0; 
    for ( units::Radian steer_angle : steer_angles){  
        command.steeringAngle = steer_angle; 
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
    return result_states; 
}

//updateNodeState 
// void HybridAstarPlanner::updateNodeState(NodeState state, NodeState current_node_state){
//     //TODO: update the node states here 
// }

//setVehicleModel 
void HybridAstarPlanner::setVehicleModel(const vehicle::SimpleCar& car_model){
    _carModel = &car_model; 
}

//verifyNodeState 
//verifies that the node state is valid. 
// A valid node-state is one that 
// (i)  does not collide with any obstacle on the static map 
// TODO: implement this for now
bool HybridAstarPlanner::verifyNodeState(NodeState state) const{
    // this state is said to be valid if the point lies within 
    // the 2D floor Grid. 
    map::GridPoint gridpoint; 
    gridpoint.row = state.yId; 
    gridpoint.column = state.xId; 
    bool result = ( _occupancyMap->isFree(gridpoint) && 
                    _occupancyMap->isWithin(gridpoint) ); 
    return result;                     
}

// updateNodeStateInfo 
// // updates the node info associated with this state. 
// void HybridAstarPlanner::updateNodeStateInfo(NodeState state, NodeStateInfo state_info){
//     std::string hash = getHash(state); 
//     _nodeInfo[hash].info = state_info; 
// }

// getOrientationResolution 
units::Radian HybridAstarPlanner::getOrientationResolution() const {
    return _orientationResolution; 
}

// getSteeringResolution 
units::Radian HybridAstarPlanner::getSteeringResolution() const {
    return _steeringResolution; 
}

// getResolution 
units::Meter HybridAstarPlanner::getResolution() const {
    return _occupancyMap->getResolution(); 
}

// getName 
std::string HybridAstarPlanner::getName()const{
    return _name; 
}

// 
void showStateOnScreen(planner::NodeState state){
    std::cout <<"NodeState {"; 
    std::cout <<"---> { "<< " xId: "<< state.xId 
                        << " yId: "<< state.yId 
                        << " thetaId: " << state.orientationId 
                        << " steeringId: "<< state.steeringId << std::endl;
    std::cout <<"--> drive state : " ; 
    showStateOnScreen(state.info.driveState); 
    std::cout << "}----------------------------"<<std::endl;
}

void showStateOnScreen(units::Radian angle){
    units::Degree degree = metricmath::radianToDegree(angle); 
    std::cout <<"theta: "<<angle.value<<"[rad] ("<<degree.value<<" [deg])"<<std::endl;
}

void showStateOnScreen(vehicle::Pose pose){
    std::cout <<"Pose: "; 
    std::cout << "x: "<<pose.x.value<<", y: "<<pose.y.value<<", ";
    showStateOnScreen(pose.orientation); 
    
}

void showStateOnScreen(vehicle::DriveState state){
    std::cout <<"DriveState {"<<std::endl; 
    std::cout <<"---> ";
    showStateOnScreen(state.pose); 
    std::cout <<"---> steeringAngle: ";
    showStateOnScreen(state.steeringAngle);
    std::cout <<"}"<<std::endl;             
}


void writePlannerToOutput(std::ofstream& outputfile, const HybridAstarPlanner& pathplanner){
    units::Meter grid_resolution = pathplanner.getResolution(); 
    units::Radian orientation_resolution = pathplanner.getOrientationResolution();
    // write tthe resolutions  
    outputfile << "[planner-"<<pathplanner.getName() << " ]"<<std::endl; 
    outputfile << grid_resolution.value << " " << orientation_resolution.value << std::endl;
    outputfile << "[plan]"<<std::endl; 
    std::vector<vehicle::DriveState> plan = pathplanner.getPlan();
    // write the number of plan points 
    outputfile << plan.size() << std::endl; 
    // write the states in the plan 
    for (vehicle::DriveState state : plan){
        outputfile << state.pose.x.value << " "
                << state.pose.y.value << " "
                << state.pose.orientation.value << " "
                << state.steeringAngle.value << std::endl; 
    }
}

void writeMapToOutput(std::ofstream& outputfile, const map::StaticObstaclesMap& static_map){
    geometry::Dimension dimension = static_map.getDimension(); 
    outputfile << "[map]" << std::endl; 
    // write the dimension of the map. 
    outputfile << dimension.width.value << " "<<dimension.height.value << std::endl; 
    
    std::vector<map::Obstacle> obstacles = static_map.getObstacles(); 
    // write the number of obstacles
    outputfile << obstacles.size() << std::endl; 
    // write each obstacles 
    for (map::Obstacle obstacle : obstacles ){
        geometry::BoundingBox box = obstacle.getBoundingBox(); 
        // write the points of the box: <top-left-x> <top-left-y> <bottom-right-x> <bottom-right-y> 
        outputfile << box.topLeft.x.value << " " << box.topLeft.y.value << " "
                   << box.bottomRight.x.value << " " << box.bottomRight.y.value <<std::endl; 
    }


}

void writeVehicleToOutput(std::ofstream& outputfile, const vehicle::SimpleCar& vehicle_model){
    units::Meter width = vehicle_model.getWidth(); 
    units::Meter length = vehicle_model.getLength(); 
    units::Radian max_steering = vehicle_model.getMaxSteering(); 
    outputfile << "[vehicle]"<<std::endl; 
    outputfile << width.value << " " << length.value<< " "<<max_steering.value << std::endl; 
    
}

void writePlanToFile(std::string filename,const HybridAstarPlanner& pathplanner,
                                          const map::StaticObstaclesMap& static_map,
                                          const vehicle::SimpleCar& vehicle_model ){
    std::ofstream outputfile;
    outputfile.open(filename); 
    if ( outputfile.is_open() ){
        writeVehicleToOutput(outputfile,vehicle_model); 
        writeMapToOutput(outputfile,static_map); 
        writePlannerToOutput(outputfile, pathplanner); 
    }

}


}//namespace planner 