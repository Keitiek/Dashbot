#include <stdlib.h>
#include <IBusBM.h>

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
  Serial.print("incoming value is ");
  Serial.println(value);
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
    String receivedString = Serial.readStringUntil('\n');
    receivedString.trim();

    int integerValue = atoi(receivedString.c_str());

    Serial.print("integerValue: ");
    Serial.println(integerValue);
    
    if (integerValue == 0) {
      Serial.println("stop!");
      currentSpeed.left = 0;
      currentSpeed.right = 0;
      sendSpeedSignalToMotors();
      delay(10);
    } else if (0 < integerValue < 100) {
      Serial.println("FORWARD!");
      forward(integerValue);
      sendSpeedSignalToMotors();
      delay(10);
    } else {
      Serial.println("Unknown command");
    }
  }  
}
