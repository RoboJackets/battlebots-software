#ifndef Telemetry_h
#define Telemetry_h

#include "Arduino.h"
#include "SD.h"
#include "../../src/Eigen/Dense.h"
#include "../../src/Eigen/Eigen337.h"

#define ACCX_TELEM  1
#define ACCY_TELEM  2
#define ACCZ_TELEM  3
#define ACCC_TELEM  4

#define MAGA_TELEM  5
#define MAGB_TELEM  6

#define IR_TELEM    7

#define MOTOR_TELEM 8

#define HEKF_TELEM  15

#define MISC_TELEM  20


class Telemetry {

    public:
        String  dir;

        Telemetry();
        ~Telemetry();

        uint8_t writeData(unsigned long time_us, Matrix<float, -1, 1> data, uint8_t telem_type);
        uint8_t writeMsg(unsigned long time_us, String msg, uint8_t telem_type);
};



#endif
