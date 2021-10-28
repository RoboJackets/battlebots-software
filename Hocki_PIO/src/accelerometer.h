
#ifndef PROGRAMS_H
#define PROGRAMS_H
#include "Controller.h"
#include "ControllerPacket.h"
#include "ADXL375.h"
#include "DriveTrain.h"
#include "PinDefs.h"
#include "ADXL375.h"
#include "AccelReading.h"
#include "Logger.h"


void setup();
void loop();

#define SPIRATE 5000000

//SPISettings spi_settings(SPIRATE, MSBFIRST, SPI_MODE3);

ADXL375 accel1(CS1, SPIRATE);
ADXL375 accel2(CS2, SPIRATE);
ADXL375 accel3(CS3, SPIRATE);
ADXL375 accel4(CS4, SPIRATE);

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

    Serial.begin(115200);
    //while(!Serial){}
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

    Logger dataLog("log.txt");
    dataLog.log("Test", "Test successful");
    dataLog.log("Overload test");
    dataLog.log("Int Test", 10);
    dataLog.log("Float test", 3.1415);



}

void loop() {
    val = accel1.getXYZ();
    Serial.print(val.z);
    Serial.print("\t");
    val = accel2.getXYZ();
    Serial.print(val.z);
    Serial.print("\t");
    val = accel3.getXYZ();
    Serial.print(val.z);
    Serial.print("\t");
    val = accel4.getXYZ();
    Serial.println(val.z);
    delay(50); 
}

#endif

