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
#include <Eigen.h>
#include <Eigen/Core>

class KalmanTest
{
    int count = 0;
    KalmanFilter k;
    int measuredVelocity = 0;
    float periodOfKalmanFilter = .01;
    float angluarAccelerationSingular = 2.0;

    KalmanTest()
    {
        k = KalmanFilter();
        Serial.begin(115200);
    }

    int main()
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
    }
};

#endif