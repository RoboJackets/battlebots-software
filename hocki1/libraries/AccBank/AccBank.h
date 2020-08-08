#ifndef AccBank_h
#define AccBank_h

#include "Arduino.h"
#include "../ADXL375/ADXL375.h"
#include "SPI.h"
#include "../../src/Eigen/Dense.h"
#include "../../src/Eigen/Eigen337.h"
#include "../../src/HEKF/HEKF.h"

#define MOSIPIN 11
#define MISOPIN 12
#define SCLKPIN 13
#define SPIRATE 5000000 // 5 MHz SPI Clock

#define CS1 14
#define CS2 15
#define CS3 9
#define CS4 10

#define INT1A 40
#define INT1B 41
#define INT2A 16
#define INT2B 17
#define INT3A 8
#define INT3B 7
#define INT4A 25
#define INT4B 24

#define GRAVITY_FORCE 9.81
#define MG_TO_G_CONV 0.001

class AccBank {

    public:

        // definition variables
        uint8_t pairs1[6];
        uint8_t pairs2[6];
        float   accangs[4];
        float   pairsdists[6];
        float   pairsangs[6];
        bool    self_testing;
        ADXL375 accs[4];

        Matrix<float, 6, 1> centproj1y;
        Matrix<float, 6, 1> centproj1x;
        Matrix<float, 6, 1> centproj2y;
        Matrix<float, 6, 1> centproj2x; 
        

        AccBank();

        void begin();
        void killAcc(HEKF* filter, uint8_t accNum);
        void reviveAcc(HEKF* filter, uint8_t accNum);
        float[6] getCentMeas();

        

}

#endif
