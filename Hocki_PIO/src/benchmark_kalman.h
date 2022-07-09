
#ifndef PROGRAMS_H
#define PROGRAMS_H
#include "RemoteController.h"
#include "ControllerPacket.h"
#include "ADXL375.h"
#include "DriveTrain.h"
#include "PinDefs.h"
#include "KalmanFilter.h"
#include <stdio.h>
#include <stdlib.h>

void setup();
void loop();
double sum = 0;
int count = 0;
Kalman k;
int measuredVelocity = 0;
float periodOfKalmanFilter = .01;
float angluarAccelerationSingular = 2.0;
void setup()
{
    // put y our setup code here, to run once:
    k = Kalman();
    Serial.begin(115200);
}

void loop()
{
    while (count < 10000)
    {
        Eigen::Vector4f angularAcceleration(2.0, 2.0, 2.0, 2.0);
        Eigen::Vector4f noise = Eigen::Vector4f::Random();

        Eigen::Vector4f measured = angularAcceleration + noise;

        Eigen::Vector2f result = k.update(measured, 0.1);

        float angle = result(0);
        float angularVelocity = result(1);

        count++;

        SerialUSB.print("Iteration: ");
        SerialUSB.print(count);
        SerialUSB.print("Angle: ");
        SerialUSB.print(angle);
        SerialUSB.print("Angular Velocity: ");
        SerialUSB.print(angularVelocity);

        measuredVelocity = (angluarAccelerationSingular * periodOfKalmanFilter);
    }
    SerialUSB.print("Final Average: ");
    SerialUSB.print(sum / count);
}

#endif
