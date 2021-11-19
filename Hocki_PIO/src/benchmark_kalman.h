
#ifndef PROGRAMS_H
#define PROGRAMS_H
#include "Controller.h"
#include "ControllerPacket.h"
#include "ADXL375.h"
#include "DriveTrain.h"
#include "PinDefs.h"
#include "KalmanFilter.h"
#include <stdio.h>
#include <stdlib.h>
#include <chrono>

void setup();
void loop();
double sum = 0;
int count = 0;
Kalman k;
void setup() {
    // put y our setup code here, to run once:
    k = Kalman();
    Serial.begin(115200);
}

void loop() {
    while (count < 10000) {
        Eigen::Vector4f measured = Eigen::Vector4f::Random();

        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        k.update(measured, 0.1);
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

        std::chrono::duration<float> fs = end - begin;

        SerialUSB.print("OP Took: ");
        SerialUSB.print(fs.count());
        sum = sum + fs.count();
        count++;
    }
    SerialUSB.print("Final Average: ");
    SerialUSB.print(sum / count);
}

#endif
