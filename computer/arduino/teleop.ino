#include <ros.h>
#include <std_msgs/String.h>
#include <IBusBM.h>

ros::NodeHandle nh;
std_msgs::String ros_msg;
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

void setup() {
  Serial.begin(baudRate);
  pinMode(cruisePinRight, OUTPUT); 
  pinMode(cruisePinLeft, OUTPUT); 
  Serial.println("Arduino is running");

  currentSpeed.left = 0;
  currentSpeed.right = 0;
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

void loop() {
  if (Serial.available() > 0) {
    //byte buffer[1];
    // Serial.readBytes(buffer, 1);
    char receivedChar = Serial.read();
    
    // char receivedChar = (char)buffer[0];
    
    Serial.print("Received Char: ");
    Serial.println(receivedChar);

    if (receivedChar == 'i') {
      Serial.println("FORWARD!");
      forward(15);
      sendSpeedSignalToMotors();
      delay(10);
    } else {
      Serial.println("stop!");
      currentSpeed.left = 0;
      currentSpeed.right = 0;
      sendSpeedSignalToMotors();
      delay(10);
    }
  }  
  nh.spinOnce();
}
