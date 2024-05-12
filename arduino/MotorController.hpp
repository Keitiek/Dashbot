#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "PinDefinitions.hpp"

// the signal values sent to left and right motors (0 - 255)
struct Signals {
    int left;
    int right;
};

void setMotorsPowers(int left, int right, Signals& signals);
void sendSpeedSignalToMotors(const Signals& signals);

#endif // MOTOR_CONTROLLER_H
