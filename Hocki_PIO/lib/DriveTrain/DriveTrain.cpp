#include <Arduino.h>
#include "DriveTrain.h"


DriveTrain::DriveTrain(int pinL, int pinR) : pinL{pinL}, pinR{pinR} {
}

void DriveTrain::init() {
    pinMode(pinL, OUTPUT);
    pinMode(pinR, OUTPUT);
    analogWriteResolution(12);
    analogWriteFrequency(pinL, F_PWM);
    analogWriteFrequency(pinR, F_PWM);

    writeESC(pinL, 500);
    writeESC(pinR, 500);
}

void DriveTrain::setPower(int powerLeft, int powerRight){
    writeESC(pinL, powerLeft);
    writeESC(pinR, powerRight);
}

/*
Arms ESC according to https://raw.githubusercontent.com/bitdump/BLHeli/master/BLHeli_32%20ARM/BLHeli_32%20manual%20ARM%20Rev32.x.pdf 
*/
void DriveTrain::arm(){
    for(int i = ESC_STOP_VAL; i < ESC_STOP_VAL + ESC_ARM_PEAK_VAL; i++) {
        writeESC(pinL, i);
        writeESC(pinR, i);
        delay(10);
    }
    delay(100);
    for(int i = ESC_STOP_VAL + ESC_ARM_PEAK_VAL; i >= ESC_STOP_VAL; i--) {
        writeESC(pinL, i);
        writeESC(pinR, i);
        delay(10);
    }
}

/* 
Writes command to ESC using PWM. 0 is full backwards, 1000 is full forwards, 500 is stopped.
This is consistent with the DSHOT command range (in case we decide to switch to DSHOT I guess)
*/
void DriveTrain::writeESC(int pin, int cmd){
  //int val = (1000+cmd) * PWM_RESOLUTION / (T_PWM*1000);
  int val = map(cmd, 0, 1000, 1638, 3276);
  analogWrite(pin, val);
}