#include <iostream>
#include <map>
#include <unordered_map>  
#include <vector> 
#include <queue>
#include <set>
#include <string> 
#include <cmath> 
#include <stack> 
#include <algorithm> 
#include <fstream> 

#include "VehicleModel.h" 
#include "Map.h"
#include "Units.h"
#include "MetricMath.h" 

#ifndef PLANNER_H
#define PLANNER_H 

namespace planner{

const double MAX_COST = 100000;
const units::Meter MAX_DIST{1000};
const std::string EMPTY_HASH  = "EMPTY_HASH";

struct NodeStateInfo{    
    vehicle::DriveState driveState;     
    double totalCost = 0; 
};


struct NodeState{
    int xId = 0; 
    int yId = 0; 
    int orientationId = 0; 
    int steeringId = 0; 
    int directionId = 1; // forward = 1, backward = 0; 
    std::string parentStateHash = EMPTY_HASH; 
    NodeStateInfo info; 
};

struct PlanningCommand {
    units::Second timeStep =  units::Second {1}; 
    units::Radian steeringAngle = units::Radian{0}; 
    units::MeterPerSecond speed = units::MeterPerSecond{1}; 
    
};


struct GreaterThanByTotalCostFromStart{

    bool operator()(const NodeState& lhs, const NodeState& rhs ) const;
};


class HybridAstarPlanner{
    public: 
    HybridAstarPlanner(const map::OccupancyMap& occupancy_map,
                        units::Radian orientation_resolution,
                        units::Radian steering_resolution); 
    void setVehicleModel(const vehicle::SimpleCar& car_model); 
    void makeplan(vehicle::DriveState start, vehicle::DriveState goal);
    virtual bool isGoalState(NodeState state, NodeState goal) const ; 
    bool isProcessed(NodeState state) const; 
    NodeState popFromQueueTop();
    void markAsProcessed(NodeState state); 
    virtual std::string getHash(NodeState state) const; 
    virtual std::vector<NodeState> getReachableNodeStates(NodeState current_state) const; 
    NodeState getNextStateAfterMove(vehicle::DriveState drivestate, PlanningCommand command) const ; 
    void pushIntoQueue(NodeState state);
    void processGoalState(NodeState goal) const; 
    void initializeQueue(); 
    bool isQueueEmpty() const; 
    NodeState getNodeState(vehicle::DriveState start) const; 
    // NodeState getNodeStateWithInfo(vehicle::DriveState drivestate) const; 
    int getOrientationId(units::Radian orientation) const; 
    int getSteeringId(units::Radian steering_angle) const; 
    bool verifyNodeState(NodeState state) const;
    virtual double computeCostOfReachingStateNode(NodeState from_node , NodeState to_node, PlanningCommand command) const; 
    virtual double computeHeuristicCostOfStateNode(NodeState state) const; 
    units::Meter computeEuclideanDistanceFromPose(vehicle::Pose pose1, vehicle::Pose pose2) const; 
    void buildPath(NodeState state); 
    NodeState lookUpNodeStateByHash(std::string state_hash) const; 
    void setStartNodeState(NodeState startNodeState); 
    void setGoalNodeState(NodeState goalNodeState); 
    bool isPathFound()const; 
    std::vector<vehicle::DriveState> getPlan() const; 

    units::Radian getOrientationResolution() const; 
    units::Radian getSteeringResolution() const; 
    units::Meter getResolution() const; 
    std::string getName() const;  

    protected:
    const vehicle::SimpleCar* _carModel = nullptr; 
    const map::OccupancyMap* _occupancyMap = nullptr; 
    std::string _name = ""; 
    units::Radian _orientationResolution = metricmath::degreeToRadian(units::Degree{20}); 
    units::Radian _steeringResolution = metricmath::degreeToRadian(units::Degree{5 });  

    NodeState _startNodeState; 
    NodeState _goalNodeState; 
    std::vector<vehicle::DriveState> _path; 
    std::priority_queue<NodeState,std::vector<NodeState>,GreaterThanByTotalCostFromStart> _priorityQueue;
    std::map<std::string,NodeState> _nodeInfo;     

};

void writePlanToFile(std::string filename,const HybridAstarPlanner& pathplanner,
                    const map::StaticObstaclesMap& static_map,
                     const vehicle::SimpleCar& vehicle_model ); 

void writeMapToOutput(std::ofstream& outputfile,const map::StaticObstaclesMap& static_map); 
void writePlannerToOutput(std::ofstream& outputfile, const HybridAstarPlanner& pathplanner); 
void writeVehicleToOutput(std::ofstream& outputfile, const vehicle::SimpleCar& vehicle_model); 

void showStateOnScreen(NodeState state);
void showStateOnScreen(units::Radian angle);
void showStateOnScreen(vehicle::Pose pose);
void showStateOnScreen(vehicle::DriveState state);

}//namespace planner 

#endif 