
#include "VehicleModel.h" 
#include "cmath"
#include "MetricMath.h"


namespace vehicle{

/**
 * SimpleCar
 * 
 * */

//Constructor 
SimpleCar::SimpleCar(SimpleCarDimension dimension, SimpleCarSteeringConfig steering_config):
_dimension(dimension),_steering_config(steering_config){}

//getLength
units::Meter SimpleCar::getLength() const{
    return _dimension.length; 
}
// getWidth 
units::Meter SimpleCar::getWidth() const{
    return _dimension.width; 
}
//getMaxSteering 
units::Radian SimpleCar::getMaxSteering() const{
    return _steering_config.maxSteering; 
}
//getInstantSteering 
units::Radian SimpleCar::getInstantSteering() const{
    return _steering_config.instantSteering; 
}

//steerBy 
units::Radian SimpleCar::steerBy(units::Radian steer, units::Radian current_steering_angle) const{
    // returns the new steering angle after a steer. 
    
    // first: makesure the steer is not more than the instantaneous steer. 
    // if it exceeds, then steer by the instantaneous steer.
    double use_steer = steer.value; 
    if ( use_steer > _steering_config.instantSteering.value ){
        use_steer = _steering_config.instantSteering.value; 
    }
    if (use_steer < -_steering_config.instantSteering.value){
        use_steer = -_steering_config.instantSteering.value; 
    }
    //compute new steering angle and peg it between 
    // -max_steering_angle <= new_steering_angle <= max_steering_angle
    double new_steering_angle = current_steering_angle.value + use_steer;
    double max_steering_angle = _steering_config.maxSteering.value;  
    if (new_steering_angle > max_steering_angle){
        new_steering_angle = max_steering_angle; 
    }
    if (new_steering_angle < -max_steering_angle){
        new_steering_angle = -max_steering_angle; 
    }
    return units::Radian{new_steering_angle}; 
}
//getNextStateAfterMove
DriveState SimpleCar::getNextStateAfterMove(DriveState current_state, units::MeterPerSecond forward_speed,
                                 units::Radian steer,units::Second time_step) const {
    // compute steering angle
    units::Radian steering_angle = steerBy(steer,current_state.steeringAngle); 
    DriveState new_state;
    if (std::abs(steering_angle.value) < 0.001){
        new_state = moveWithVerySmallSteeringAngle(current_state,forward_speed,steering_angle,time_step);
    }else{
        new_state = moveWithSteeringAngle(current_state,forward_speed,steering_angle,time_step);
    }
    
    return new_state; 
}

DriveState SimpleCar::moveWithVerySmallSteeringAngle(DriveState current_state, units::MeterPerSecond forward_speed,
                                 units::Radian steering_angle,units::Second time_step) const {
    // compute distance to be moved 
    double distance = forward_speed.value * time_step.value; 
    
    Pose pose = current_state.pose; 
    DriveState new_state;     
    new_state.pose.x = units::Meter{pose.x.value + distance * std::cos(pose.orientation.value)};
    new_state.pose.y = units::Meter{pose.y.value + distance * std::sin(pose.orientation.value)};
    new_state.pose.orientation = metricmath::addToOrientation(pose.orientation,steering_angle);
    new_state.steeringAngle = steering_angle; 

    return new_state; 
}

DriveState SimpleCar::moveWithSteeringAngle(DriveState current_state, units::MeterPerSecond forward_speed,
                                 units::Radian steering_angle,units::Second time_step) const{
    // compute distance to be moved 
    double distance = forward_speed.value * time_step.value; 
    // comute turning angle and turning radius     
    double turning_angle = distance * std::tan(steering_angle.value) / _dimension.length.value; 
    double turning_radius = distance / turning_angle; 
    // compute center of curvature 
    Pose pose = current_state.pose; 
    double cx = pose.x.value - std::sin(pose.orientation.value) * turning_radius;
    double cy = pose.y.value + std::cos(pose.orientation.value) * turning_radius; 
    // compute new position 
    Pose new_pose; 
    new_pose.x = units::Meter{cx + std::sin(pose.orientation.value + turning_angle) * turning_radius}; 
    new_pose.y = units::Meter{cy - std::cos(pose.orientation.value + turning_angle) * turning_radius}; 
    new_pose.orientation = metricmath::addToOrientation(pose.orientation,units::Radian{turning_angle});

    DriveState new_state; 
    new_state.pose = new_pose; 
    new_state.steeringAngle = units::Radian{steering_angle};  

    return new_state;   
}


}// namespace vehicle 