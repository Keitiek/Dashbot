#include "DifferentialDriveKinematics.hpp"
#include <Arduino.h>

Speeds DifferentialDriveKinematics(float linear_x, float angular_z) {

    float wheelbase = 0.543;
    float v = linear_x;
    float w = angular_z;
    float l = wheelbase;
        
    float v_left = v - (w * l/2);
    float v_right = v + (w * l/2);
    
    Speeds speeds = {v_left, v_right};
    return speeds;
}
