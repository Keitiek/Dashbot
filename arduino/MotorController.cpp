#include "MotorController.hpp"
#include "PinDefinitions.hpp"
#include <Arduino.h>

// reverse is acting strange, it seems like it's always the same power no matter the incoming values
void setMotorsPowers(int left, int right, Signals& signals) {
    signals.left = map(left, 0, 100, 0, 255);
    signals.right = map(right, 0, 100, 0, 255);

    if (signals.left >= 0) {
        digitalWrite(cruisePinLeft, LOW);
    } else {
        digitalWrite(cruisePinLeft, HIGH);
    }
    if (signals.right >= 0) {
        digitalWrite(cruisePinRight, LOW);
    } else {
        digitalWrite(cruisePinRight, HIGH);
    }
    sendSpeedSignalToMotors(signals);
    delay(10);
}

void sendSpeedSignalToMotors(const Signals& signals) {
    analogWrite(throttlePinLeft, signals.left);
    analogWrite(throttlePinRight, signals.right);
}
