#include "src/Eigen/Eigen337.h"
#include "src/Eigen/Dense.h"
#include "src/HEKF/HEKF.h"
#include "libraries/ADXL375/ADXL375.h"
#include "libraries/ADXL375/AccelReading.h"
#include "libraries/AccBank/AccBank.h"
#include "SPI.h"
using namespace Eigen;

// DEFINE HEKF PARAMS //
#define HEKFDT 1.25e-4 // 5 update steps per control loop iteration ideally
#define ALPHA  1.3775
#define BETA   62.0224
#define BOTR   0.0953
#define WHLR   0.0381

// DEFINE PINS - MAGS //
#define MAGA  38
#define MAGB  39
#define SRPIN 32

#define MAGTS 5e-6

// DEFINE PINS - IR RECVS //
#define INT10K 23
#define INT20K 22
#define PK10K  20
#define PK20K  21
#define REF10K 18
#define REF20K 19
#define IRSEL  37

// control variables
unsigned long ta;
unsigned long tb;
unsigned long t0;
HEKF *filter;
AccBank *accbank;
elapsedMicros controlTimer;
elapsedMicros hekfClock;
float leftV, rightV;

void setup() {

  // begin serial output
  Serial.begin(9600);
  
  // set up HEKF Filter
  filter = initHEKF(HEKFDT, ALPHA, BETA, WHLR, BOTR);

  // begin accelerometers
  accbank = new AccBank();
  (*accbank).begin();

  // begin "motors"
  leftV = 0;
  rightV = 0;

  // begin time itself
  hekfClock = 0;
  controlTimer = 0;
}

void control() {

  float spinV = (leftV + rightV) / 2;
  updateHEKF(filter, (*accbank).getCentMeas(), spinV, hekfClock, ACC);
  
}


void loop() {
  if(controlTimer >= 1000) { // update every 1 ms
    controlTimer = controlTimer - 1000;
    control();
  }
}
