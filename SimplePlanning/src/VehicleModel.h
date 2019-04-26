#include <iostream> 
#include <vector> 
#include "Units.h"


#ifndef VEHICLEMODEL_H
#define VEHICLEMODEL_H 

namespace vehicle{

struct Pose{
    units::Meter x{0}; 
    units::Meter y{0}; 
    units::Radian orientation{0}; 
};

struct SimpleCarDimension{
    units::Meter length{0.2}; 
    units::Meter width{0.1}; 
};

struct PoseState{
    Pose pose; 
};

struct DriveState{
    Pose pose; 
    units::Radian steeringAngle{0}; 

};

// class DifferentialDrive{
//     public:
//     // returns the next PoseState after executing a move, using the 
//     // differential drive motion model. 
//     DriveState getNextStateAfterMove(PoseState current_state, double forward_speed, double steer) const ;

// };

struct SimpleCarSteeringConfig{
    units::Radian maxSteering{15 * 3.141 / 180.0}; 
    units::Radian instantSteering{ 15 * 3.141 / 180.0}; // 15deg (default) 
};

class SimpleCar{
    public:
    SimpleCar(SimpleCarDimension dimension, SimpleCarSteeringConfig steering); 
    DriveState getNextStateAfterMove(DriveState current_state,
             units::MeterPerSecond forward_speed,
             units::Radian steer,
             units::Second time_step) const ;  
    units::Radian steerBy(units::Radian steer,
                            units::Radian current_steering_angle) const ; 
    units::Meter getLength()const ; 
    units::Meter getWidth() const; 

    units::Radian getMaxSteering() const; 
    units::Radian getInstantSteering() const; 

    DriveState moveWithSteeringAngle(DriveState current_state, units::MeterPerSecond forward_speed,
                                 units::Radian steering_angle,units::Second time_step) const;   

    DriveState moveWithVerySmallSteeringAngle(DriveState current_state, units::MeterPerSecond forward_speed,
                                 units::Radian steering_angle,units::Second time_step) const; 

    private:    
    SimpleCarDimension _dimension; 
    SimpleCarSteeringConfig _steering_config; 
};


}// namespace vehicle




#endif 