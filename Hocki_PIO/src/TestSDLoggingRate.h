#ifndef PROGRAM_H
#define PROGRAM_H

#include <Arduino.h>
#include "Logger.h"
#include "PinDefs.h"

Logger accelLog;

void setup()
{
    Serial.begin(115200);
    while(!Serial);
    SPI.begin();
    accelLog.begin("Log.txt");

    int startTime = micros();
    for(int i = 0; i < 20; i++)
    {
        accelLog.log("0");
    }
    accelLog.flush();
    Serial.print("Logging took: ");
    Serial.print(micros() - startTime);
}

void loop()
{
    delay(1);
}

#endif