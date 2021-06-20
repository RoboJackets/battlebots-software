#ifndef AccBank_h
#define AccBank_h

#include <Arduino.h>
#include <PinDefs.h> 
#include <Hocki.h>
#include <SPI.h>
#include <eigen/include/Eigen/Eigen>
#include "ADXL375.h"
//#include "../../src/HEKF/HEKF.h"

using namespace Eigen;

#define SPIRATE 5000000 // 5 MHz SPI Clock

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
        ADXL375* accs[4];
        float ofsx[4]; 
        float ofsy[4]; 
        float ofsz[4]; 

        Matrix<float, 6, 1> centproj1y;
        Matrix<float, 6, 1> centproj1x;
        Matrix<float, 6, 1> centproj2y;
        Matrix<float, 6, 1> centproj2x; 
        
        AccBank();
        ~AccBank();

        void begin();
        //void killAcc(HEKF* filter, uint8_t accNum);
        //void reviveAcc(HEKF* filter, uint8_t accNum);
        void toggleSelfTest();
        Matrix<float, 6, 1> getCentMeas();
        Matrix<float, 4, 1> getXMeas();
        Matrix<float, 4, 1> getYMeas();
        Matrix<float, 4, 1> getZMeas();
        Matrix<float, 4, 3> getAllMeas(); 

};

#endif
