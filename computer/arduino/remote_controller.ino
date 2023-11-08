#include <IBusBM.h>

// channels on physical RC:
// 1 - left (negative), right (positive)
// 3 - break (negative), throttle (positive)
// 5 - SWD up (negative), SWD down (positive)
// 6 - LIGHTS OFF (negative), ON (positive)
// 8 - FWD (negative), REV (positive)

const long baudRate = 115200;

const int throttlePin = 2; 
const int throttlePinFaster = 11;
const int motorPin1 = 7;
const int motorPin2 = 5;

const int leftRightChannel = 0;
const int breakThrottleChannel = 2;
const int lightsOffOnChannel = 5; // (-100, 100)
const int forwardReverseChannel = 7; // (-100, 0, 100)

const int numberOfChannelsWeAreInterestedIn = 7;

IBusBM ibus;

// Read the number of a given channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t readValue = ibus.readChannel(channelInput);
  if (readValue < 100) return defaultValue;
  return map(readValue, 1000, 2000, minLimit, maxLimit);
}

// Read the channel and return a boolean value
bool readSwitch(byte channelInput, bool defaultValue) {
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int readValue = readChannel(channelInput, 0, 100, intDefaultValue);
  return (readValue > 50);
}

void setup() {
  Serial.begin(baudRate);
  pinMode(motorPin1, OUTPUT); 
  pinMode(motorPin2, OUTPUT); 
  // Attach iBus object to serial port
  ibus.begin(Serial1);
}

void loop() {
  for (byte channel = 0; channel <= numberOfChannelsWeAreInterestedIn; channel++) {
    int value = readChannel(channel, -100, 100, 0);

  // Throttle and break
    if(channel == breakThrottleChannel){
      if(value > 10){ // throttle // todo: the greater the throttle value, the more speed should be added
        forward();
      } else { // speed to 0 // todo: break so that it stops even if going downhill
        breakk();
      } 

  // Turning left and right
    } else if(channel == leftRightChannel){
        if(value < -50){
        turnLeft();
      } if(value > 50) {
        turnRight();
      } 
  // Driving forward or reversing
    } else if(channel == forwardReverseChannel) {
      if(value > 0){
        reverse();
      } else if(value == 0) {
        Serial.print("NEUTRAL | ");
      } else{
        Serial.print("FORWARD | ");
      }
  // Turning lights on and off
    } else if(channel == lightsOffOnChannel){
      if(value < 0){
        Serial.print("LIGHTS OFF | ");
      } else {
        Serial.print("LIGHTS ON | ");
      }
    }
  }
  Serial.println();
  delay(10);
}

void turnLeft() {
  Serial.print("LEFT | ");
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  analogWrite(throttlePin, 30);
  analogWrite(throttlePinFaster, 31);
}

void turnRight(){
  Serial.print("RIGHT | ");
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
  analogWrite(throttlePin, 30);
  analogWrite(throttlePinFaster, 31);
}

void forward(){
  Serial.print("THROTTLE | ");
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, HIGH);
  analogWrite(throttlePin, 30);
  analogWrite(throttlePinFaster, 31);  
}

void breakk(){ // this is not a typo, break() is seen as an inbuilt function so we need another function name for our code to work as intended :)
  Serial.print("BREAK | ");
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, HIGH);
  analogWrite(throttlePin, 0);
  analogWrite(throttlePinFaster, 0); 
}

void reverse(){
  Serial.print("REVERSE | ");
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  analogWrite(throttlePin, 30);
  analogWrite(throttlePinFaster, 31);
}

// todo: rewrite and use for motor speed control 
void setMotorPower(int value){
  value = constrain(value, 0, 100);
  int pwmValue = map(value, 0, 100, 0, 255);

  analogWrite(motorPin1, value);
  analogWrite(motorPin2, value);

  Serial.print("Motor power: ");
  Serial.println(value);
}
