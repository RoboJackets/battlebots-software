#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#define F_PWM 400 // Hz
#define T_PWM 2.5 // 2.5ms = 1/400 Hz
#define PWM_RESOLUTION 4096 //12 bits

#define ESC_STOP_VAL 500
#define ESC_ARM_PEAK_VAL 300

class DriveTrain{
    public:
        DriveTrain(int pinL, int pinR);
        void init();
        void arm();
        void setPower(int powerLeft, int powerRight);

    private:
        int pinL, pinR;
        void writeESC(int pin, int cmd);
};

#endif