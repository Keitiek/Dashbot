#ifndef MYFUNCTIONS_H
#define MYFUNCTIONS_H

#include <Arduino.h>

struct Speeds {
    float left;
    float right;
};

Speeds DifferentialDriveKinematics(float linear_x, float angular_z);

#endif
