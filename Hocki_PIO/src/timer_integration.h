#ifndef PROGRAM_H
#define PROGRAM_H

#include <Arduino.h>
#include "Logger.h"
#include "ADXL375.h"
#include "AccelReading.h"
#include "PinDefs.h"

#include "Controller.h"
#include "ControllerPacket.h"
#include "DriveTrain.h"

void setup();
void loop();

IntervalTimer timer;
Logger accelLog;
AccelReading val1;
AccelReading val2;
AccelReading val3;
AccelReading val4;

#define SPIRATE 5000000

ADXL375 accel1(CS1, SPIRATE);
ADXL375 accel2(CS2, SPIRATE);
ADXL375 accel3(CS3, SPIRATE);
ADXL375 accel4(CS4, SPIRATE);

Controller c;
ControllerPacket p;
DriveTrain drive(ESC_L, ESC_R);

void addLine()
{
    val1 = accel1.getXYZ();
    val2 = accel2.getXYZ();
    val3 = accel3.getXYZ();
    val4 = accel4.getXYZ();

    accelLog.addLine(val1, val2, val3, val4);
    
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

    accelLog.begin("Timer7.txt");

    timer.begin(addLine, 1000);
}

void loop()
{
    if(accelLog.dumpFlag)
    {
        accelLog.dump();
    }

    if (c.read(&p)) {
        c.wdt.feed();
        if(p.tankDrive){
            SerialUSB.print("X Speed: ");
            SerialUSB.print(p.xSpeed);
            SerialUSB.print("  Y Speed: ");
            SerialUSB.print(p.ySpeed);
            SerialUSB.print("  Rot Speed: ");
            SerialUSB.println(p.rotSpeed);
            int powerL = map(p.xSpeed, 245, 1805, 300, 700);
            int powerR = map(p.xSpeed, 245, 1805, 300, 700);
            drive.setPower(powerL, powerR);
        }
    } else {
        Serial.println("Not reading.");
    }

}

#endif //PROGRAM_H