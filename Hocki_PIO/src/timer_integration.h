#ifndef PROGRAM_H
#define PROGRAM_H

#include <Arduino.h>
#include "Logger.h"
#include "ADXL375.h"
#include "AccelReading.h"
#include "PinDefs.h"

#include "RemoteController.h"
#include "ControllerPacket.h"
#include "DriveTrain.h"
#include <FastLED.h>

#define LED_PIN 4
#define CLOCK_PIN 3
#define BUTTON_PIN 5 
#define COLOR_ORDER GBR

#define NUM_LEDS 8
CRGB leds[NUM_LEDS];      //naming our LED array
int brightness = 75;

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

float vavg;
float position = 0;

void addLine()
{
    val1 = accel1.getXYZ();
    val2 = accel2.getXYZ();
    val3 = accel3.getXYZ();
    val4 = accel4.getXYZ();

    if (BRD_VER == 1) {
        float v1 = val1.x;
        v1 = sqrt(v1 * 0.0088954f);
        float v2 = val2.y;
        v2 = sqrt(v2 * 0.0088954f);
        float v3 = val3.x;
        v3 = sqrt(v3 * 0.0088954f);
        float v4 = val4.y;
        v4 = sqrt(v4 * 0.0088954f);

        vavg = (v1 + v2 + v3 + v4) / 4;

        position += (vavg * 0.01);

        if (position > 2*PI) {
            position -= 2*PI;
        } else if (position < 0) {
            position += 2*PI;
        }
    } else if (BRD_VER == 2) {
        float v1 = sqrt(pow(val1.x, 2)  + pow(val1.y, 2)); //Get centripetal acceleration
        v1 = sqrt(v1 * 0.0088954f); // Calculate velocity from centripetal acceleration
        float v2 = sqrt(pow(val2.x, 2)  + pow(val2.y, 2));
        v2 = sqrt(v2 * 0.0088954f);
        float v3 = sqrt(pow(val3.x, 2)  + pow(val3.y, 2));
        v3 = sqrt(v3 * 0.0088954f);
        float v4 = sqrt(pow(val4.x, 2)  + pow(val4.y, 2));
        v4 = sqrt(v4 * 0.0088954f);

        vavg = (v1 + v2 + v3 + v4) / 4;

        position += (vavg * 0.01);

        if (position > 2*PI) {
            position -= 2*PI;
        } else if (position < 0) {
            position += 2*PI;
        }
    }

    if (position < PI) {
        fill_solid(leds, NUM_LEDS, CHSV(300, 100, brightness));  
        FastLED.show();
    } else {
        fill_solid(leds, NUM_LEDS, CHSV(180, 100, brightness));  
        FastLED.show();
    }

    accelLog.addLine(val1, val2, val3, val4, vavg, position);
    
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

    FastLED.addLeds<DOTSTAR, LEDD1, LEDC1, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
    FastLED.setBrightness(  brightness );

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
    //drive.init();
    //drive.arm();
	//c.init();
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