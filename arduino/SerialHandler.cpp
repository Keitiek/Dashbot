#include <Arduino.h>

#include "DifferentialDriveKinematics.hpp"
#include "SerialHandler.hpp"

Speeds processSerialInput(int& bytesRead, byte* buffer, Values& values) {
  Speeds speeds = {0.0, 0.0}; // Initialize speeds

  while (Serial.available() > 0) {
    buffer[bytesRead] = Serial.read(); // Read incoming byte
    bytesRead++;

    // Process the received data when all bytes are read
    if (bytesRead == sizeof(Values)) {
      memmove(&values, buffer, sizeof(values));

      Serial.println(values.left);
      Serial.println(values.right);
      // Check that the values are within the desired range
      if (values.left >= -2.8 && values.left <= 2.8 && values.right >= -2.8 && values.right <= 2.8) {
        // Use the received float values
        speeds.left = values.left;
        speeds.right = values.right;

        bytesRead = 0;
        return speeds;
      } else {
        //Serial.println("Values out of range, discarding...");
        bytesRead = 0;
      }
    }
  }
  return speeds;
}
