#include <IBusBM.h>
#include <stdlib.h>

#include "DifferentialDriveKinematics.hpp"
#include "FlySkyChannelDefinitions.hpp"
#include "PinDefinitions.hpp"
#include "MotorController.hpp"
#include "IBusHandler.hpp"
#include "SerialHandler.hpp"

IBusBM ibus;

const long baudRate = 115200;

static byte buffer[8]; // Buffer to store incoming data
static int bytesRead = 0; // Number of bytes read so far

// Signals is imported from MotorController, is a struct that holds
// the signal values sent to left and right motors (0 - 255)
Signals signals;

// Values is imported from SerialHandler, is a struct that holds
// the incoming speed values sent over serial connection (-2.8 to -2.8)
Values values;

// Speeds is imported from DifferentialDriveKinematics, calculates the desired speeds
// based on incoming linear x and angular z from FlySky
Speeds speeds;

void setup() {
  Serial.begin(baudRate);

  pinMode(cruisePinRight, OUTPUT);
  pinMode(cruisePinLeft, OUTPUT);

  // Attach iBus object to serial port
  ibus.begin(Serial1);

  signals.left = 0;
  signals.right = 0;
}

// braking at speed 0 happens because of BAC settings (activated in kilowatt app).
// If the throttle has a low value, the brake applies automatically.

void loop() {
  int manualAutoValue = readIBusChannel(ibus, manualAutoChannel, -100, 100, 0); // either M or A, else 0

  if (manualAutoValue == auto_channel) {
    //Values values;
    speeds = processSerialInput(bytesRead, buffer, values); // values coming in from serial communication
  }
  else { // if operating mode is Manual aka FlySky
    int brakeThrottleValue = readIBusChannel(ibus, brakeThrottleChannel, -28, 28, 0); // linear x
    int leftRightValue = readIBusChannel(ibus, leftRightChannel, -28, 28, 0); // angular z

    speeds = DifferentialDriveKinematics(brakeThrottleValue / 10.0, leftRightValue / 10.0);
  }

  Serial.print("speeds.right: ");
  Serial.println(speeds.right);
  Serial.print("speeds.left: ");
  Serial.println(speeds.left);

  // an intermediary step is needed:
  // take speeds and send corresponding signals to motors
  // for this, odometry needs to measure what signal produces what speed

  // value is created for testing purposes only
  // drives with value 6
  int value =  0; // speeds.left + 5;
  Serial.println(value);

  setMotorsPowers(value, value, signals);
  sendSpeedSignalToMotors(signals);
  delay(10);
}
