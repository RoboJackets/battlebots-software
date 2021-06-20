
#include "pins.h"
#include "hocki.h"
#include "src/Eigen/Eigen337.h"
#include "src/Eigen/Dense.h"
#include "src/HEKF/HEKF.h"
#include "libraries/ADXL375/ADXL375.h"
#include "libraries/ADXL375/AccelReading.h"
#include "libraries/AccBank/AccBank.h"
#include "libraries/Telemetry/Telemetry.h"
#include "libraries/StateLED/StateLED.h"
#include "SPI.h"
using namespace Eigen;

// control variables
unsigned long ta;
unsigned long tb;
unsigned long t0;
float targheading;
HEKF *filter;
AccBank *accbank;
Telemetry *telem;
StateLED *leds;
elapsedMicros controlTimer;
elapsedMicros hekfClock;
float leftV, rightV;

void setup() {

  // begin serial output
  Serial.begin(115200);
  
  // set up HEKF Filter
  filter = initHEKF(HEKFDT, ALPHA, BETA, WHLR, BOTR);

  // begin accelerometers
  accbank = new AccBank();
  accbank->begin();

  // begin telemetry
  telem = new Telemetry();

  // begin LEDs
  targheading = 0;
  leds = new StateLED();

  // begin "motors"
  leftV = 0;
  rightV = 0;

  // begin time itself
  hekfClock = 0;
  controlTimer = 0;
}

void control() {

  uint8_t updateStatus;
  float spinV = (leftV + rightV) / 2;
  Matrix<float, 6, 1> accMeas = accbank->getCentMeas();
  updateStatus = updateHEKF(filter, accMeas, spinV, hekfClock, ACC);
  telem->writeData(hekfClock, accMeas, ACCC_TELEM);
  telem->writeData(hekfClock, filter->x, HEKF_TELEM);
  leds->topOn(filter, false, targheading);
}


void loop() {
  if(controlTimer >= 1000) { // update every 1 ms
    controlTimer = controlTimer - 1000;
    control();
  }
}
