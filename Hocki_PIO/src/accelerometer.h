
#ifndef PROGRAMS_H
#define PROGRAMS_H
#include "Controller.h"
#include "ControllerPacket.h"
#include "ADXL375.h"
#include "DriveTrain.h"
#include "PinDefs.h"
#include "ADXL375.h"
#include "AccelReading.h"

void setup();
void loop();

#define SPIRATE 1000000

//SPISettings spi_settings(SPIRATE, MSBFIRST, SPI_MODE3);

ADXL375 accel(CS1, SPIRATE);
AccelReading val;

void setup() {
  // put your setup code here, to run once:
    pinMode(CS1, OUTPUT);
    pinMode(CS2, OUTPUT);
    pinMode(CS3, OUTPUT);
    pinMode(CS4, OUTPUT);

    digitalWrite(CS1, HIGH);
    digitalWrite(CS2, HIGH);
    digitalWrite(CS3, HIGH);
    digitalWrite(CS4, HIGH);

    Serial.begin(57600);
    while(!Serial){}
    Serial.println("Started");

    Serial.println("Init");
    accel.init();

    accel.startMeasuring();
}

void loop() {
    Serial.println("hello");
    val = accel.getXYZ();
    val.printDebug();
    delay(200); 
}

#endif

