#ifndef CHANNEL_DEFINITIONS_H
#define CHANNEL_DEFINITIONS_H

// channels on physical RC:
// 1 - left (negative), right (positive)
// 3 - break (negative), throttle (positive)
// 5 - SWD up (negative), SWD down (positive)
// 6 - LIGHTS OFF (negative), ON (positive)
// 8 - FWD (negative), REV (positive)

const int leftRightChannel = 0;
const int brakeThrottleChannel = 2;
const int manualAutoChannel = 4;
const int auto_channel = 100;       //  teleop
const int manual_channel = -100;    //  remote controller

#endif // CHANNEL_DEFINITIONS_H
