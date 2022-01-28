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

//IntervalTimer timer;
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
    //while(!Serial);

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

    //timer.begin(addLine, 1000);
    drive.init();
    //drive.arm();
	c.init();
}

void loop()
{
    if(accelLog.dumpFlag)
    {
        accelLog.dump();
    }

    if (c.read(&p)) {
        c.wdt.feed();
        Serial.print("X Speed: ");
        Serial.print(p.xSpeed);
        Serial.print("  Y Speed: ");
        Serial.print(p.ySpeed);
        Serial.print("  Rot Speed: ");
        Serial.println(p.rotSpeed);
        if(p.tankDrive){
            Serial.println("Tank Drive");
            float xScaled = map((float)p.xSpeed, 245, 1805, -1.0, 1.0);
            float rotScaled = map((float)p.ySpeed, 245, 1805, -1.0, 1.0);
            int powerL = map(xScaled - rotScaled, -2, 2, 300, 700);
            int powerR = map(xScaled + rotScaled, -2, 2, 300, 700);
            Serial.print("PowerL: ");
            Serial.println(powerL);
            Serial.print("PowerR: ");
            Serial.print(powerR);
            /*
            int powerL = map(p.xSpeed, 245, 1805, 300, 700);
            int powerR = map(p.xSpeed, 245, 1805, 300, 700);
            */
            drive.setPower(powerL, powerR);
        }
        else  //Spin mode
        {
            Serial.println("Spin Mode");
            float rotScaled = map((float)p.rotSpeed, 245, 1805, 0, 1.0);
            int powerL = map(-rotScaled, -1, 1, 300, 700);
            int powerR = map(rotScaled, -1, 1, 300, 700);
            Serial.print("PowerL: ");
            Serial.println(powerL);
            Serial.print("PowerR: ");
            Serial.print(powerR);
            drive.setPower(powerL, powerR);
        }
    } else {
        //Serial.println("Not reading.");
    }

    delay(5);
}

#endif //PROGRAM_H