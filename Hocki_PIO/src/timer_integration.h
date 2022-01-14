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

int precision = 4;
int index = 0;
int time = 0;
int times[6];
int size;
double readingA1X[6];
double readingA1Y[6];
double readingA2X[6];
double readingA2Y[6];
double readingA3X[6];
double readingA3Y[6];
double readingA4X[6];
double readingA4Y[6];


void readData() {
    times[index] = time;
    val1 = accel1.getXYZ();
    val2 = accel2.getXYZ();
    val3 = accel3.getXYZ();
    val4 = accel4.getXYZ();
    readingA1X[index] = val1.x;
    readingA1Y[index] = val1.y;
    readingA2X[index] = val2.x;
    readingA2Y[index] = val2.y;
    readingA3X[index] = val3.x;
    readingA3Y[index] = val3.y;
    readingA4X[index] = val4.x;
    readingA4Y[index] = val4.y;
    index ++;
    time ++;
    size += precision;
    size += String(time).length();
    if ((size + 11 + precision + String(time).length()) > 64) {
        writeData();
    }
}

void writeData() {
    accelLog.logStampedArray("A1X", times, readingA1X, index);
    accelLog.logStampedArray("A1Y", times, readingA1Y, index);
    accelLog.logStampedArray("A2X", times, readingA2X, index);
    accelLog.logStampedArray("A2Y", times, readingA2Y, index);
    accelLog.logStampedArray("A3X", times, readingA3X, index);
    accelLog.logStampedArray("A3Y", times, readingA3Y, index);
    accelLog.logStampedArray("A4X", times, readingA4X, index);
    accelLog.logStampedArray("A4Y", times, readingA4Y, index);
    accelLog.flush();
    index = 0;
    size = 0;
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

    accelLog.begin("Log.txt");

    timer.begin(readData, 1000);
}

void loop() {
    delay(1000);
}

#endif //PROGRAM_H