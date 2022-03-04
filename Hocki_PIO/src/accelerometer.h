
#ifndef PROGRAMS_H
#define PROGRAMS_H
#include "RemoteController.h"
#include "ControllerPacket.h"
#include "ADXL375.h"
#include "DriveTrain.h"
#include "PinDefs.h"
#include "ADXL375.h"
#include "AccelReading.h"
#include "Logger.h"
#include "KalmanFilter.h"

void setup();
void loop();

#define SPIRATE 5000000

//SPISettings spi_settings(SPIRATE, MSBFIRST, SPI_MODE3);

ADXL375 accel1(CS1, SPIRATE);
ADXL375 accel2(CS2, SPIRATE);
ADXL375 accel3(CS3, SPIRATE);
ADXL375 accel4(CS4, SPIRATE);

ADXL375* accels[4] = {&accel1, &accel2, &accel3, &accel4};

AccelReading val;

Logger dataLog;


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

    dataLog.begin("log.txt");
}

void loop() {

    for(int i = 0; i < 4; i++)
    {
        val = accels[i]->getXYZ();
        Serial.print("Accel");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(val.y);
    }
    /*
    int startTime = micros();
    for(int i = 0; i < 4; i++)
    {
      val = accel1.getXYZ();

    }
    Serial.print("Time to read accel: ");
    Serial.println(micros() - startTime); // This takes about 52 us

    startTime = micros();
    dataLog.log("Acc1Z", val.z);

    //val = accel2.getXYZ();
    dataLog.log("Acc2Z", val.z);

    //val = accel3.getXYZ();
    dataLog.log("Acc3Z", val.z);

    //val = accel4.getXYZ();
    dataLog.log("Acc4Z", val.z);

    dataLog.flush();
    Serial.print("Time to write SD: ");
    Serial.println(micros() - startTime); // This takes about 3.3ms
    */
    delay(1000); 
}

#endif

