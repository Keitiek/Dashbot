#ifndef SERIAL_HANDLER_H
#define SERIAL_HANDLER_H

#include <Arduino.h>

// for storing the resulting speeds based on values coming from serial connection from teleop keyboard
// min value -2.8 m/s, max value 2.8 m/s
struct Values {
    volatile float left;
    volatile float right;
};

Speeds processSerialInput(int& bytesRead, byte* buffer, Values& values);

#endif // SERIAL_HANDLER_H
