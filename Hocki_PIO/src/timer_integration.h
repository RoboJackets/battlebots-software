#ifndef PROGRAM_H
#define PROGRAM_H

#include <Arduino.h>
#include "Logger.h"
#include "ADXL375.h"
#include "AccelReading.h"
#include "PinDefs.h"


void setup();
void loop();
void readData();
void writeData();

IntervalTimer timer;
Logger accelLog;
AccelReading val1;
AccelReading val2;
AccelReading val3;
AccelReading val4;

#define SPIRATE 5000000

//SPISettings spi_settings(SPIRATE, MSBFIRST, SPI_MODE3);

ADXL375 accel1(CS1, SPIRATE);
ADXL375 accel2(CS2, SPIRATE);
ADXL375 accel3(CS3, SPIRATE);
ADXL375 accel4(CS4, SPIRATE);


void addLine()
{
    val1 = accel1.getXYZ();
    val2 = accel2.getXYZ();
    val3 = accel3.getXYZ();
    val4 = accel4.getXYZ();

    /* 
    val1 = AccelReading(199.99, 199.99, 199.99, 1);
    val2 = AccelReading(9.99, 9.99, 9.99, 1);
    val3 = AccelReading(99.99, 99.99, 99.99, 1);
    val4 = AccelReading(9.0, 9.0, 9.0, 1);
    */
    

    if(accelLog.lineCount < 100)
    {
        accelLog.addLine(val1, val2, val3, val4);
    }
}

void setup() {

    pinMode(CS1, OUTPUT);
    pinMode(CS2, OUTPUT);
    pinMode(CS3, OUTPUT);
    pinMode(CS4, OUTPUT);

    digitalWrite(CS1, HIGH);
    digitalWrite(CS2, HIGH);
    digitalWrite(CS3, HIGH);
    digitalWrite(CS4, HIGH);

    Serial.begin(115200);
    while(!Serial);

    SPI.begin();

    accel1.init();
    accel1.setCalibrationValue(2, -3);
    accel1.startMeasuring();

    accel2.init();
    accel2.setCalibrationValue(2, -5);
    accel2.startMeasuring();

    accel3.init();
    accel3.setCalibrationValue(2, -5);
    accel3.startMeasuring();

    accel4.init();
    accel4.setCalibrationValue(2, -5);
    accel4.startMeasuring();

    accelLog.begin("Timer4.txt");

    timer.begin(addLine, 1000);
}

void loop() {

    if(accelLog.dumpFlag)
    {
        Serial.println("Ready to Dump");
        accelLog.dump();
    }
}

#endif //PROGRAM_H