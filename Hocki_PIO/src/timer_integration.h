#ifndef PROGRAM_H
#define PROGRAM_H

#include <Arduino.h>
#include "Hocki.h"
#include "Logger.h"
#include "ADXL375.h"
#include "AccelReading.h"
#include "PinDefs.h"

#include "RemoteController.h"
#include "ControllerPacket.h"
#include "DriveTrain.h"
#include <FastLED.h>

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
DriveTrain drive(ESC_L, ESC_R, c);

float vrecord = 0;
unsigned int vcount = 1;
volatile float vcalibrate = 0;

volatile float vavg;
volatile float position = 0;

float wrapAngle(float angle)
{
    if (angle >= 2*PI) {
        angle -= 2*PI;
    } else if (angle < 0) {
        angle += 2*PI;
    }
    return angle;
}

void addLine()
{
    val1 = accel1.getXYZ();
    val2 = accel2.getXYZ();
    val3 = accel3.getXYZ();
    val4 = accel4.getXYZ();

    if (BRD_VER == 1) {
        
        float v1 = val1.x*9.8f;
        v1 = sqrt(fabs(v1 / 0.0088954f));
        float v2 = val2.y*9.8f;
        v2 = sqrt(fabs(v2 / 0.0088954f));
        float v3 = val3.x*9.8f;
        v3 = sqrt(fabs(v3 / 0.0088954f));
        float v4 = val4.y*9.8f;
        v4 = sqrt(fabs(v4 / 0.0088954f));
        /*
        float rotation = sqrt(2)/2.0;

        float s = 0.01258f;

        float v1 = sqrt(fabs((rotation*val3.x - rotation*val3.y)/9.8f - (rotation*val2.x - rotation*val2.y)/9.8f)/s); //x3 - x2
        float v2 = sqrt(fabs(-(rotation*val3.x + rotation*val3.y)/9.8f + (rotation*val4.x + rotation*val4.y)/9.8f)/s); //y3 + y4
        float v3 = sqrt(fabs((rotation*val1.x + rotation*val1.y)/9.8f - (rotation*val2.x + rotation*val2.y)/9.8f)/s); //y1 - y2
        float v4 = sqrt(fabs((rotation*val1.x - rotation*val1.y)/9.8f - (rotation*val4.x - rotation*val4.y)/9.8f)/s); //x1 - x4
`       */
        vavg = (v1 + v2 + v3 + v4) / 4.0f - vcalibrate;

        position += (vavg * 0.001);

        position = wrapAngle(position);
        //Serial.printf("v1: %f, v2: %f, v3: %f, v4: %f, vavg: %f, position: %f\n", v1, v2, v3, v3,vavg, position);
    } else if (BRD_VER == 2) {
        /*
        float v1 = sqrt(pow(val1.x, 2)  + pow(val1.y, 2)); //Get centripetal acceleration
        v1 = sqrt(v1 * 0.0088954f); // Calculate velocity from centripetal acceleration
        float v2 = sqrt(pow(val2.x, 2)  + pow(val2.y, 2));
        v2 = sqrt(v2 * 0.0088954f);
        float v3 = sqrt(pow(val3.x, 2)  + pow(val3.y, 2));
        v3 = sqrt(v3 * 0.0088954f);
        float v4 = sqrt(pow(val4.x, 2)  + pow(val4.y, 2));
        v4 = sqrt(v4 * 0.0088954f);
        */
    
        float s = 0.01258f;
        float v1 = sqrt(fabs(val4.x*9.8f - val1.x*9.8f)/s);
        float v2 = sqrt(fabs(val4.y*9.8f - val3.y*9.8f)/s);
        float v3 = sqrt(fabs(val1.y*9.8f - val2.y*9.8f)/s);
        float v4 = sqrt(fabs(val3.x*9.8f - val2.x*9.8f)/s);

        //vavg = (v1 + v2 + v3 + v4) / 4 - 9.2690;

        //vavg = ((v1 + v2 + v3 + v4) / 4) - vcalibrate;
        vavg = ((v1 + v2 + v3 + v4) / 4.0f);
        

        position += (vavg * 0.001);

        position = wrapAngle(position);
    }


    float time = millis() / 1.0f;
    //Serial.println(position);

    accelLog.addLine(val1, val2, val3, val4, vavg, position);
    
}

void setup() {

    pinMode(CS1, OUTPUT);
    pinMode(CS2, OUTPUT);
    pinMode(CS3, OUTPUT);
    pinMode(CS4, OUTPUT);

    pinMode(13, OUTPUT);

    digitalWrite(CS1, HIGH);
    digitalWrite(CS2, HIGH);
    digitalWrite(CS3, HIGH);
    digitalWrite(CS4, HIGH);

    digitalWrite(13, HIGH);

    Serial.begin(115200);
    //while(!Serial);

    if (BRD_VER == 1) {
        pinMode(LEDD1, OUTPUT);
        digitalWrite(LEDD1, HIGH);
    }
    else if (BRD_VER == 2) {
        FastLED.addLeds<DOTSTAR, LEDD1, LEDC1, COLOR_ORDER> (leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
        FastLED.setBrightness(  brightness );
    }

    SPI.begin();

    accel1.init();
    accel1.setCalibrationValue(0, -2);
    accel1.setCalibrationValue(2, -2);
    accel1.setCalibrationValue(1, -1);
    accel1.startMeasuring();

    accel2.init();
    accel2.setCalibrationValue(0, -1);
    accel2.setCalibrationValue(1, -2);
    accel2.setCalibrationValue(2, 0);
    accel2.startMeasuring();

    accel3.init();
    accel3.setCalibrationValue(0, -3);
    accel3.setCalibrationValue(1, 0);
    accel3.setCalibrationValue(2, -6);
    accel3.startMeasuring();

    accel4.init();
    accel4.setCalibrationValue(0, -2);
    accel4.setCalibrationValue(1, 2);
    accel4.setCalibrationValue(2, -3);
    accel4.startMeasuring();

    accelLog.begin("Multi_Speed_Uncalibrated.txt");


    timer.begin(addLine, 1000);
	c.init();
    drive.init();
    drive.arm();
}


void loop()
{
    if(accelLog.dumpFlag)
    {
        accelLog.dump();
    }
    
    
    if (c.read(&p)) {
        c.wdt.feed();
        /*
        Serial.print("X Speed: ");
        Serial.print(p.xSpeed);
        Serial.print("  Y Speed: ");
        Serial.print(p.ySpeed);
        Serial.print("  Rot Speed: ");
        Serial.println(p.rotSpeed);
        */
        if (p.calibrate) {
            vcount ++;
            vrecord += vavg;
            if (BRD_VER == 2) {
                fill_solid(leds, NUM_LEDS, CHSV(85, 128, brightness));
            }
        } 
        else if(p.tankDrive){
            
            vcalibrate = vrecord/vcount;
            //Serial.println("Tank Drive");
            float xScaled = map((float)p.xSpeed, 245, 1805, -1.0, 1.0);
            float rotScaled = map((float)p.ySpeed, 245, 1805, -0.5, 0.5);
            int powerL = map(xScaled - rotScaled, -2, 2, 300, 700);
            int powerR = map(xScaled + rotScaled, -2, 2, 300, 700);
            //Serial.printf("%d, %d\n", powerL, powerR);
            /*
            Serial.print("PowerL: ");
            Serial.println(powerL);
            Serial.print("PowerR: ");
            Serial.print(powerR);
            */
            
            drive.setPower(powerL, powerR);
            if (BRD_VER == 2) {
                fill_solid(leds, NUM_LEDS, CHSV(0, 128, brightness));
            }
        }
        else  //Spin mode
        {
            
            vcalibrate = vrecord/vcount;
            //Serial.println("Spin Mode");
            float rotScaled = map((float)p.rotSpeed, 245, 1805, 0, 1.0);
            float xScaled = map((float)p.xSpeed, 245, 1805, -1.0, 1.0);
            float yScaled = map((float)p.ySpeed, 245, 1805, -1.0, 1.0);


            int powerL, powerR;
            if (abs(xScaled) > 0.1 && abs(yScaled) > 0.1) {
                float theta_D = wrapAngle(atan2(yScaled, xScaled) + 5*PI/4); //Add Pi/2 + 3Pi/4 to account for LED placement
                //Serial.println(theta_D);

                if (theta_D < PI) // Angle doesn't get wrapped
                {
                    if (position > theta_D && position < theta_D + PI)
                    {
                        powerL = map(-rotScaled, -1, 1, 300, 700);
                        powerR = map(rotScaled * T_POWER_RATIO, -1, 1, 300, 700);
                    }
                    else
                    {
                        powerL = map(-rotScaled * T_POWER_RATIO, -1, 1, 300, 700);
                        powerR = map(rotScaled, -1, 1, 300, 700);
                    }
                }
                else //Angle does get wrapped
                {
                    if (position > theta_D || position < wrapAngle(theta_D + PI))
                    {
                        powerL = map(-rotScaled, -1, 1, 300, 700);
                        powerR = map(rotScaled * T_POWER_RATIO, -1, 1, 300, 700);
                    }
                    else
                    {
                        powerL = map(-rotScaled * T_POWER_RATIO, -1, 1, 300, 700);
                        powerR = map(rotScaled, -1, 1, 300, 700);
                    }
                }
            } else {
                powerL = map(-rotScaled, -1, 1, 300, 700);
                powerR = map(rotScaled, -1, 1, 300, 700);
            }
            if(p.reversed)
            {
                drive.setPower(powerR, powerL);
            }
            else
            {
                drive.setPower(powerL, powerR);
            }
            
            
            if (position < PI/2 || position > 3*PI/2) {
                if (BRD_VER == 1) {
                    digitalWrite(LEDD1, HIGH);
                } else {
                    fill_solid(leds, NUM_LEDS, CHSV(100, 128, brightness));
                }
            } else {
                if (BRD_VER == 1) {
                    digitalWrite(LEDD1, LOW);
                } else if (BRD_VER == 2) { 
                    fill_solid(leds, NUM_LEDS, CHSV(200, 128, brightness));
                }
            }
            //Serial.printf("%f, %f, %d, %d\n",vavg, position, powerL, powerR);
        }
        
    } else {
        //Serial.println("Not reading.");
    }
    //Serial.println(vavg);
    FastLED.show();
    delay(5);
}

#endif //PROGRAM_H