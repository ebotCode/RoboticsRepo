#include <iostream> 
#include <assert.h> 

#include "Map.h"
#include "Geometry.h" 
#include "VehicleModel.h" 
#include "Display.h"
#include "MetricMath.h" 
#include "PlannerVariant.h" 



void testVehicleSteering(){
    vehicle::SimpleCarSteeringConfig steeringConfig;
    steeringConfig.maxSteering = units::Radian{15 * 3.141 / 180.0};
    steeringConfig.instantSteering = units::Radian{15 * 3.141 / 180.0};

    vehicle::SimpleCarDimension dimension; 
    dimension.length = units::Meter{0.2}; 
    vehicle::SimpleCar car(dimension,steeringConfig); 
    //try steer 
    units::Radian current_steering{0}; 
    units::Radian steer{100 * 3.141 / 180.0}; 
    units::Radian new_angle = car.steerBy(steer,current_steering);
    std::cout <<" After steering "<<metricmath::radianToDegree(current_steering).value<<
                        " ---> "<< metricmath::radianToDegree(new_angle).value<<std::endl;

}   

void testVehicleMove(){
    vehicle::SimpleCarSteeringConfig steeringConfig;
    steeringConfig.maxSteering = units::Radian{15 * 3.141 / 180.0};
    steeringConfig.instantSteering = units::Radian{15 * 3.141 / 180.0};

    vehicle::SimpleCarDimension dimension; 
    dimension.length = units::Meter{0.2}; 
    vehicle::SimpleCar car(dimension,steeringConfig); 
    // Define a drive state
    vehicle::DriveState state; 
    state.pose.x = units::Meter{0};
    state.pose.y = units::Meter{0}; 
    state.pose.orientation = metricmath::degreeToRadian(units::Degree{45}); 
    state.steeringAngle = units::Radian{0}; 
    //try steer and move. 
    units::MeterPerSecond velocity{1}; 
    units::Radian steer = metricmath::degreeToRadian(units::Degree{5}); 
    units::Second time_step{0.1}; 
    vehicle::DriveState new_state = car.getNextStateAfterMove(state,velocity,steer,time_step);
    vehicle::DriveState new_state2 = car.getNextStateAfterMove(new_state,velocity,steer,time_step); 
    
    std::cout<<"Before Moving........."<<std::endl; 
    display::print(state); 
    std::cout<<"After Moving.........."<<std::endl;
    std::cout<<"new state"<<std::endl; 
    display::print(new_state); 
    std::cout<<"new state2 "<<std::endl;
    display::print(new_state2); 

}  


 
int main(){


    testVehicleSteering();
    testVehicleMove(); 

}