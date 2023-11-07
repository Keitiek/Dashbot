/*
  Arduino FS-I6X Demo
  fsi6x-arduino-mega-ibus.ino
  Read iBus output port from FS-IA6B receiver module
  Display values on Serial Monitor

  Channel functions by Ricardo Paiva - https://gist.github.com/werneckpaiva/

  DroneBot Workshop 2021
  https://dronebotworkshop.com
*/

// Include iBusBM Library
#include <IBusBM.h>

const int throttlePin = 2; // Choose the PWM pin you want to use
const int throttlePinFaster = 11; // Choose the PWM pin you want to use

const int motorPin1 = 2;
const int motorPin2 = 11;

// Create iBus Object
IBusBM ibus;

// Read the number of a given channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = ibus.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

// Read the channel and return a boolean value
bool readSwitch(byte channelInput, bool defaultValue) {
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}

void setup() {
//  pinMode(motorPin1, OUTPUT);
//  pinMode(motorPin2, OUTPUT);
  // Start serial monitor
  Serial.begin(115200);
  pinMode(7, OUTPUT); // Set pin 7 as output
  pinMode(5, OUTPUT); // Set pin 5 as output
  // Attach iBus object to serial port
  ibus.begin(Serial1);
}

void loop() {
  // analogWrite(throttlePinReverse, HIGH);
  // Cycle through first 5 channels and determine values
  // Print values to serial monitor
  // Note IBusBM library labels channels starting with "0"

  for (byte i = 0; i < 5; i++) {
    int value = readChannel(i, -100, 100, 0);

//Edasi, stop ja tagasi
    
    if(i == 2 && value > 10){
      Serial.print("THROTTLE");
      digitalWrite(7, HIGH);
      digitalWrite(5, HIGH);
      analogWrite(throttlePin, 30);
      analogWrite(throttlePinFaster, 31);
    }
    if(i == 2 && value == 0){
      Serial.print("STOP");
      digitalWrite(7, HIGH);
      digitalWrite(5, HIGH);
      analogWrite(throttlePin, 0);
      analogWrite(throttlePinFaster, 0);
    }
    if(i == 2 && value < -10){
      Serial.print("REVERSE");
      digitalWrite(7, LOW);
      digitalWrite(5, LOW);
      analogWrite(throttlePin, 30);
      analogWrite(throttlePinFaster, 31);
    }

// Pööramine

    if(i == 3 && value > 50){
      Serial.print("LEFT");
      digitalWrite(7, LOW);
      digitalWrite(5, HIGH);
      analogWrite(throttlePin, 30);
      analogWrite(throttlePinFaster, 31);
    }
    
    if(i == 3 && value < -50){
     Serial.print("RIGHT");
     digitalWrite(7, HIGH);
     digitalWrite(5, LOW);
     analogWrite(throttlePin, 30);
     analogWrite(throttlePinFaster, 31);
    }

// Serial Monitor/Plotter
      
    Serial.print("Ch");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(value);
    Serial.print(" | ");
  }
//
//  setMotorPower(value);
//
//  delay(1000);

  // Print channel 6 (switch) boolean value
  Serial.print("Ch6: ");
  Serial.print(readSwitch(5, false));
  Serial.println();

  delay(10);
}

void setMotorPower(int value){
  value = constrain(value, 0, 100);

  int pwmValue = map(value, 0, 100, 0, 255);

  analogWrite(motorPin1, value);
  analogWrite(motorPin2, value);


  Serial.print("Motor power: ");
  Serial.println(value);
}