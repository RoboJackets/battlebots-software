#include <Arduino.h>
#include "AccBank.h"
#include "ADXL375.h"
#include "SPI.h"

AccBank::AccBank() {

    self_testing = false;

    // set up SPI for Accelerometers
    SPI.begin();
    SPI.setMOSI(MOSIPIN);
    SPI.setMISO(MISOPIN);
    SPI.setSCK(SCLKPIN);

    int p1[6] = {1, 1, 1, 2, 2, 3};
    int p2[6] = {2, 3, 4, 3, 4, 4};
    memcpy(pairs1, p1, sizeof(pairs1));
    memcpy(pairs2, p2, sizeof(pairs2));

    float _accangs[4] = {0.7854, 0.7854, 0.7854, 0.7854};
    float _pairsangs[6] = {2.3562, 3.1416, -2.3562, -2.3562, -1.5708, -0.7854};
    float _pairsdists[6] = {0.0257, 0.0363, 0.0257, 0.0257, 0.0363, 0.0257};
    memcpy(accangs, _accangs, sizeof(accangs));
    memcpy(pairsangs, _pairsangs, sizeof(pairsangs));
    memcpy(pairsdists, _pairsdists, sizeof(pairsdists));

    for(int k = 0; k < 6; k++) {
        centproj1y(k, 0) = sin(pairsangs[k] + accangs[pairs1[k]]);
        centproj1x(k, 0) = cos(pairsangs[k] + accangs[pairs1[k]]);
        centproj2y(k, 0) = sin(pairsangs[k] + accangs[pairs2[k]]);
        centproj2x(k, 0) = cos(pairsangs[k] + accangs[pairs2[k]]);
    }

    // set up acc interrupt pins
    pinMode(INT1A, INPUT);
    pinMode(INT1B, INPUT);
    pinMode(INT2A, INPUT);
    pinMode(INT2B, INPUT);
    pinMode(INT3A, INPUT);
    pinMode(INT3B, INPUT);
    pinMode(INT4A, INPUT);
    pinMode(INT4B, INPUT);

    // offsets determined by user in m/s^2
    float ofsx[4] = {0, 0, 0, 0};
    float ofxy[4] = {0, 0, 0, 0};
    float ofsz[4] = {0, 0, 0, 0};

    accs[0] = ADXL375(CS1, SPIRATE);
    accs[1] = ADXL375(CS2, SPIRATE);
    accs[2] = ADXL375(CS3, SPIRATE);
    accs[3] = ADXL375(CS4, SPIRATE);

}

AccBank::~AccBank() {
    delete[] pairs1;
    delete[] pairs2;
    delete[] accangs;
    delete[] pairsdists;
    delete[] pairsangs;
    for(uint8_t k = 0; k < 4; k++) {
        accs[k].stop();
        delete accs[k];
    }
}

void AccBank::begin() {
    for(uint8_t k = 0; k < 4; k++) 
        accs[k].init();
        accs[k].startContinuousOperation(ofxs[k], ofsy[k], ofsz[k]);
    }
}

/*
void AccBank::killAcc(HEKF* filter, uint8_t accNum) {
    for(uint8_t k = 0; k < 6; k++) {
        if(pairs1[k] == accNum || pairs2[k] == accNum) {
            killSensor(filter, ACC, k);
        }
    }
}

void AccBank::reviveAcc(HEKF* filter, uint8_t accNum) {
    for(uint8_t k = 0; k < 6; k++) {
        if(pairs1[k] == accNum || pairs2[k] == accNum) {
            reviveSensor(filter, ACC, k);
        }
    }
}
*/
void toggleSelfTest() {
    if(self_testing) {
        self_testing = !self_testing;
        for(uint8_t k = 0; k < 4; k++) {
            accs[k].deactivateSelfTest();
        }
    }
    else {
        self_testing = !self_testing;
        for(uint8_t k = 0; k < 4; k++) {
            accs[k].activateSelfTest();
        }
    }
}

Matrix<float, 4, 3> AccBank::getAllMeas() {
    Matrix<float, 4, 3> ret;
    AccelReading readk
    for(uint8_t k = 0; k < 4; k++) {
        readk = accs[k].getXYZ();
        ret(k, 0) = readk.x * GRAVITY_FORCE * MG_TO_G_CONV;
        ret(k, 1) = readk.y * GRAVITY_FORCE * MG_TO_G_CONV;
        ret(k, 2) = readk.z * GRAVITY_FORCE * MG_TO_G_CONV;
    }
    return ret;
}

Matrix<float, 4, 1> AccBank::getXMeas() {
    Matrix<float, 4, 1> ret;
    Matrix<float, 4, 3> allret = getAllMeas();
    for(uint8_t k = 0; k < 4; k++) {
        ret(k, 0) = allret(k, 0);
    }
    return ret;
}

Matrix<float, 4, 1> AccBank::getYMeas() {
    Matrix<float, 4, 1> ret;
    Matrix<float, 4, 3> allret = getAllMeas();
    for(uint8_t k = 0; k < 4; k++) {
        ret(k, 0) = allret(k, 1);
    }
    return ret;
}

Matrix<float, 4, 1> AccBank::getZMeas() {
    Matrix<float, 4, 1> ret;
    Matrix<float, 4, 3> allret = getAllMeas();
    for(uint8_t k = 0; k < 4; k++) {
        ret(k, 0) = allret(k, 2);
    }
    return ret;
}

Matrix<float, 6, 1> AccBank::getCentMeas() {
    Matrix<float, 6, 1> ret;
    Matrix<float, 4, 3> allret = getAllMeas();
    uint8_t idx1, idx2;
    float cg1, cg2;
    for(uint8_t k = 0, k < 6; k++) {
        idx1 = pairs1(k, 0);
        idx2 = pairs2(k, 0);
        cg1 = allret(idx1, 0) * centproj1x(k, 0) + allret(idx1, 1) * centproj1y(k, 0);
        cg2 = allret(idx2, 0) * centproj2x(k, 0) + allret(idx2, 1) * centproj2y(k, 0);
        ret(k, 0) = cg2 - cg1;
    }
    ret.cwiseAbs();
    return ret;
}
