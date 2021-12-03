#include <Arduino.h>
#include <iostream>

IntervalTimer testTimer;
const int ledPin = 13;//LED_BUILTIN;
int ledState = LOW;

void test() {
    if (ledState == LOW) {
        ledState = HIGH;
    }
    else {
        ledState = LOW;
    }
    digitalWrite(ledPin, ledState);
}

void setup() {
    pinMode(ledPin, OUTPUT);
    Serial.begin(9600);
    testTimer.begin(test, 1000);
}

void loop() {
    delay(1000);
}

