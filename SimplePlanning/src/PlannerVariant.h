#include "Planner.h" 

#ifndef PLANNER_VARIAHT_H
#define PLANNER_VARIANT_H 

namespace planner{

class HybridAstarWithObstaclePressure : public HybridAstarPlanner{
    public:
    HybridAstarWithObstaclePressure(const map::OccupancyMap& occupancy_map,
                        units::Radian orientation_resolution,
                        units::Radian steering_resolution);

    double computeHeuristicCostOfStateNode(NodeState state) const; 
    double computeProximityCost(vehicle::Pose) const; 
    double computeCostOfReachingStateNode(NodeState from_node , NodeState to_node, PlanningCommand command) const; 
    bool isGoalState(NodeState state, NodeState goal) const ; 
    std::string getHash(NodeState state) const; 
    std::vector<NodeState> getReachableNodeStates(NodeState current_state) const; 
    // std::vector<NodeState> HybridAstarWithObstaclePressure::getReachableNodeStates(NodeState parent_state) const;
};

} // namespace planner 

#endif 