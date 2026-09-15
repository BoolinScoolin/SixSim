#include <Arduino.h>
#include "hal/fcu/teensytest/timer.hpp"

sixsim::hal::TeensyTestTimer timer;

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.print("Timer reading: ");
    Serial.println(timer.read());
}

void loop() 
{
    delay(1000);
    Serial.print("Timer reading: ");
    Serial.println(timer.read());
}
