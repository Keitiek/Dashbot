#include <IBusBM.h>

// channels on physical RC:
// 1 - left (negative), right (positive)
// 3 - break (negative), throttle (positive)
// 5 - SWD up (negative), SWD down (positive)
// 6 - LIGHTS OFF (negative), ON (positive)
// 8 - FWD (negative), REV (positive)

const long baudRate = 115200;

struct Speed {
  int left;
  int right;
};

Speed currentSpeed;

const int throttlePinRight = 2; 
const int throttlePinLeft = 11;
const int cruisePinRight = 7;
const int cruisePinLeft = 5;

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

void setup() {
  Serial.begin(baudRate);
  pinMode(cruisePinRight, OUTPUT); 
  pinMode(cruisePinLeft, OUTPUT); 
  // Attach iBus object to serial port
  ibus.begin(Serial1);

  currentSpeed.left = 0;
  currentSpeed.right = 0;
}

int lastTurningValue = 0;
int currentTurningValue = 0;

void loop() {  
  for (byte channel = 0; channel <= numberOfChannelsWeAreInterestedIn; channel++) {
    
    int value = readChannel(channel, -100, 100, 0);
    
    // Throttle and break
    if(channel == breakThrottleChannel){
      if (value >= 0){
        forward(value);
      } 
    } 

     if (channel == leftRightChannel){
      if (value != 0) {
        adjustTurning(value);
        sendSpeedSignalToMotors();
      }
    }
     
    // Driving forward or reversing
    /*
    if(channel == forwardReverseChannel) {
      if(value > 0){
        reverse();
      } else if(value == 0) {
        Serial.print("NEUTRAL | ");
      } else{
        Serial.print("FORWARD | ");
      }
    }
    */
    // Turning lights on and off 

    /*
    if(channel == lightsOffOnChannel){
      if(value < 0){
       // Serial.print("LIGHTS OFF | ");
      } else {
        Serial.print("LIGHTS ON | ");
      }
    }
    */
  }
  sendSpeedSignalToMotors();
  delay(10);
}

void adjustTurning(int value) {
  int turnSpeedLeft = 0;
  int turnSpeedRight = 0;
  if(value < 0){
    turnSpeedLeft = map(value, 0, -100, 0, 255);
    currentSpeed.right += (turnSpeedLeft );
  } else {
    turnSpeedRight = map(value, 0, 100, 0, 255);
    currentSpeed.left += (turnSpeedRight);
  }
    
  currentSpeed.left = constrain(currentSpeed.left, 0, 255);
  currentSpeed.right = constrain(currentSpeed.right, 0, 255);
}

void forward(int value){
  int leftSpeed = map(value, 0, 100, 0, 255);
  int rightSpeed = map(value, 0, 100, 0, 255);
  currentSpeed.right = rightSpeed; 
  currentSpeed.left = leftSpeed;
}

void sendSpeedSignalToMotors(){
  digitalWrite(cruisePinRight, LOW);
  digitalWrite(cruisePinLeft, LOW);
  
  Serial.print("RIGHT value | ");
  Serial.println(currentSpeed.right);

  Serial.print("LEFT value | ");
  Serial.println(currentSpeed.left);
  
  analogWrite(throttlePinRight, currentSpeed.right);
  analogWrite(throttlePinLeft, currentSpeed.left);  
}

// braking at speed 0 happens because of BAC settings (activated in kilowatt app). 
// If the throttle has a low value, the break applies automatically.
 
// todo: modify to use reverse
void reverse(int value){
  Serial.print("REVERSE | ");
  int leftSpeed = map(value, 0, -100, 0, -255);
  int rightSpeed = map(value, 0, -100, 0, -255);
  currentSpeed.right = rightSpeed; 
  currentSpeed.left = leftSpeed;
}

// next steps in hardware: wires repairs, jetson pocket, waterproof the electronics, base for cargo, cargo ties 
// next steps in code: apply reverse, teleop: control over ros, attach camera
