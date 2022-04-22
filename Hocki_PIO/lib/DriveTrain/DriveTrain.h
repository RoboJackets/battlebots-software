#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "RemoteController.h"

#define F_PWM 4000 // Hz
#define T_PWM 0.25 // .250 ms = 1/400 Hz
#define PWM_RESOLUTION 4096 //12 bits

#define ESC_STOP_VAL 500
#define ESC_ARM_PEAK_VAL 300

#define MIN_PULSE 0.125
#define MAX_PULSE 0.25

class DriveTrain{
    public:
        DriveTrain(int pinL, int pinR, Controller c);
        void init();
        void arm();
        void setPower(int powerLeft, int powerRight);

    private:
        int pinL, pinR;
        Controller control;
        void writeESC(int pin, int cmd);
};

#endif