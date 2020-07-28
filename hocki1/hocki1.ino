#include "src/Eigen/Eigen337.h"
#include "src/Eigen/Dense.h"
#include "src/HEKF/HEKF.h"
#include "SPI.h"
using namespace Eigen;

// DEFINE HEKF PARAMS //
#define HEKFDT 1.25e-4 // 5 update steps per control loop iteration ideally
#define ALPHA  1.3775
#define BETA   62.0224
#define BOTR   0.0953
#define WHLR   0.0381

// DEFINE PINS - SPI //
#define MOSIPIN 11
#define MISOPIN 12
#define SCLKPIN 13
#define SPIRATE 3000000 // 3 MHz SPI clock-rate

// DEFINE PINS - ACCS //
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

#define XAXIS 1
#define YAXIS 2
#define ZAXIS 3

#define ACCSF 0.4807

#define ACCNAN 3000 // this acceleration will never be read, so it can represent a missing value

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

SPISettings hockiSPISettings(SPIRATE, MSBFIRST, SPI_MODE3);

// control variables
unsigned long ta;
unsigned long tb;
unsigned long t0;
volatile HEKF* filter;
unsigned long microControlPeriod = 625; // 1600 Hz Update Clock or something
float rightVolt, leftVolt;

// acc variables
Matrix<int, 6, 1> pairs1;
Matrix<int, 6, 1> pairs2; 
Matrix<float, 6, 1> pairsdists;
Matrix<float, 6, 1> pairsangs;
Matrix<float, 4, 1> accangs; 
Matrix<float, 6, 1> centproj1y;
Matrix<float, 6, 1> centproj1x;
Matrix<float, 6, 1> centproj2y;
Matrix<float, 6, 1> centproj2x;
Matrix<float, 4, 1> accReadingsX;
Matrix<float, 4, 1> accReadingsY;
Matrix<float, 4, 1> accReadingsZ;

const byte acc_reset[2][18] = { 
  {
    0x1D, // address for THRESH_SHOCK
    0x1E, // address for OFSX
    0x1F, // address for OFSY
    0x20, // address for OFSZ
    0x21, // address for DUR
    0x22, // address for Latent
    0x23, // address for Window
    0x24, // address for THRESH_ACT,
    0x25, // address for THRESH_INACT
    0x26, // address for TIME_INACT
    0x27, // address for ACT_INACT_CTL
    0x2A, // address for SHOCK_AXES
    0x2C, // address for BW_RATE
    0x2D, // address for POWER_CTL, 
    0x2E, // address for INT_ENABLE 
    0x2F, // address for INT_MAP
    0x31, // address for DATA_FORMAT, 
    0x38, // address for FIFO_CTL
  },
  {
    0x00, // setting for THRESH_SHOCK
    0x00, // setting for OFSX
    0x00, // setting for OFSY
    0x00, // setting for OFSZ
    0x00, // DUR
    0x00, // Latent
    0x00, // Window
    0x00, // THRESH_ACT
    0x00, // THRESH_INACT
    0x00, // TIME_INACT
    0x00, // ACT_INACT_CTL
    0x00, // SHOCK_AXES
    0x0D, // BW_RATE: No "Low Power" and Rate = 1.6khz
    0x08, // POWER_CTL: Enable Measurement mode
    0x81, // INT_ENABLE: Enable DATA_READY and Overrun interrupts
    0x01, // INT_MAP: Send DATA_READY to INT1 and Overrun to INT2
    0x0B, // DATA_FORMAT: Right justified
    0x00, // FIFO_CTL: Completely disable FIFO buffer
  }
};

// mag variables



void setup() {

  // begin serial
  Serial.begin(9600);

  // begin time itself
  t0 = micros();
  
  // set up chip selects
  pinMode(CS1, OUTPUT); digitalWrite(CS1, HIGH);
  pinMode(CS2, OUTPUT); digitalWrite(CS2, HIGH);
  pinMode(CS3, OUTPUT); digitalWrite(CS3, HIGH);
  pinMode(CS4, OUTPUT); digitalWrite(CS4, HIGH);

  // set up acc interrupt pins
  pinMode(INT1A, INPUT);
  pinMode(INT1B, INPUT);
  pinMode(INT2A, INPUT);
  pinMode(INT2B, INPUT);
  pinMode(INT3A, INPUT);
  pinMode(INT3B, INPUT);
  pinMode(INT4A, INPUT);
  pinMode(INT4B, INPUT);

  // set up mag pins
  pinMode(MAGA, INPUT);
  pinMode(MAGB, INPUT);
  pinMode(SRPIN, OUTPUT);

  // set up IR pins
  pinMode(IRSEL, OUTPUT); digitalWrite(IRSEL, LOW);
//  pinMode(

  // set up SPI
  SPI.begin();
  SPI.setMOSI(MOSIPIN);
  SPI.setMISO(MISOPIN);
  SPI.setSCK(SCLKPIN);

  // set up HEKF Filter
  filter = initHEKF(HEKFDT, ALPHA, BETA, WHLR, BOTR);

  // set up acc linear algebra matrices
  pairs1 << 1, 1, 1, 2, 2, 3;
  pairs2 << 2, 3, 4, 3, 4, 4;
  pairsdists << 0.0257, 0.0363, 0.0257, 0.0257, 0.0363, 0.0257;
  pairsangs << 2.3562, 3.1416, -2.3562, -2.3562, -1.5708, -0.7854;
  accangs << 0.7854, 0.7854, 0.7854, 0.7854;
  for(int k = 0; k < 6; k++) {
    centproj1y(k, 0) = sin(pairsangs(k, 0) + accangs(pairs1(k, 0), 0));
    centproj1x(k, 0) = cos(pairsangs(k, 0) + accangs(pairs1(k, 0), 0));
    centproj2y(k, 0) = sin(pairsangs(k, 0) + accangs(pairs2(k, 0), 0));
    centproj2x(k, 0) = cos(pairsangs(k, 0) + accangs(pairs2(k, 0), 0));
  }
  accReadingsX.setConstant(ACCNAN);
  accReadingsY.setConstant(ACCNAN);
  accReadingsZ.setConstant(ACCNAN);

  // set up acc sensors via SPI
  for(int k = 0; k < 4; k++) {
    writeAcc(k, acc_reset[0], acc_reset[1], 18);
  }

  // set up ACC interrupts using DATA_READY ACC signal
  attachInterrupt(INT1A, acc1ISR, RISING); SPI.usingInterrupt(INT1A);
  attachInterrupt(INT2A, acc2ISR, RISING); SPI.usingInterrupt(INT2A);
  attachInterrupt(INT3A, acc3ISR, RISING); SPI.usingInterrupt(INT3A);
  attachInterrupt(INT4A, acc4ISR, RISING); SPI.usingInterrupt(INT4A);

  // init motor voltages
  rightVolt = 0;
  leftVolt = 0;
  
}



void loop() {
  noInterrupts();
  ta = micros();

  // pull acc data if not done by interrupts
  int pin;
  for(byte k = 0; k < 4; k++) {
    switch(k) {
      case 0:
        pin = INT1A;
        break;
      case 1:
        pin = INT2A;
        break;
      case 2:
        pin = INT3A;
        break; 
      case 3:
        pin = INT4A;
        break;
      default:
        break;
    }
    if((accReadingsX(k, 0) == ACCNAN || accReadingsY(k, 0) == ACCNAN || accReadingsZ(k, 0) == ACCNAN) && digitalRead(pin) == HIGH) {
      accReadingsX(k, 0) = readAcc(k, XAXIS);
      accReadingsY(k, 0) = readAcc(k, YAXIS);
      accReadingsZ(k, 0) = readAcc(k, ZAXIS);
    }
  }

  // preprocess accelerometer signal - sork's mastapeece
  Matrix<float, 6, 1> centGuess;
  byte idx1, idx2;
  float centGuess2, centGuess1;
  for(byte k = 0; k < 6; k++) {
    idx1 = pairs1(k, 0);
    idx2 = pairs2(k, 0);
    centGuess2 = accReadingsX(idx2, 0) * centproj2x(k, 0) + accReadingsY(idx2, 0) * centproj2y(k, 0);
    centGuess1 = accReadingsX(idx1, 0) * centproj1x(k, 0) + accReadingsY(idx1, 0) * centproj1x(k, 0);
    centGuess(k, 0) = centGuess2 - centGuess1;
  }
  centGuess.cwiseAbs();
  
  // filter acc data
  float spinVolt = (leftVolt + rightVolt) / 2;
  updateHEKF(filter, centGuess, spinVolt, ta-t0, ACC);

  // controls go here ok use filter->x or something idk

  tb = micros();
  interrupts();
  if(microControlPeriod > (tb - ta)) { // pls pls pls never be false lol
    delayMicroseconds(microControlPeriod - (tb - ta)); 
  }
}

// ISRs // 

void acc1ISR() {
  accReadingsX(0, 0) = readAcc(0, XAXIS);
  accReadingsY(0, 0) = readAcc(0, YAXIS);
  accReadingsZ(0, 0) = readAcc(0, ZAXIS);
}

void acc2ISR() {
  accReadingsX(1, 0) = readAcc(1, XAXIS);
  accReadingsY(1, 0) = readAcc(1, YAXIS);
  accReadingsZ(1, 0) = readAcc(1, ZAXIS);
}

void acc3ISR() {
  accReadingsX(2, 0) = readAcc(2, XAXIS);
  accReadingsY(2, 0) = readAcc(2, YAXIS);
  accReadingsZ(2, 0) = readAcc(2, ZAXIS);
}

void acc4ISR() {
  accReadingsX(3, 0) = readAcc(3, XAXIS);
  accReadingsY(3, 0) = readAcc(3, YAXIS);
  accReadingsZ(3, 0) = readAcc(3, ZAXIS);  
}

void magISR() {
  
}

// Helpers //

VectorXf magfiltfilt(int taps, VectorXf b, VectorXf a, VectorXf sig, 

void writeAcc(byte numAcc, const byte address[], const byte data[], int numWrites) {
  int pin;
  switch(numAcc) {
    case 0:
      pin = CS1;
      break;
    case 1:
      pin = CS2;
      break;
    case 2:
      pin = CS3;
      break; 
    case 3:
      pin = CS4;
      break;
    default:
      return;
  }
  int transm;
  for(int k = 0; k < numWrites; k++) {
    transm = (0 << 15) | (0 << 14) | (((int) address[k]) << 8) | ((int) data[k]);
    SPI.beginTransaction(hockiSPISettings);
    digitalWrite(pin, LOW);
    SPI.transfer16(transm);
    digitalWrite(pin, HIGH); 
    SPI.endTransaction();
  }
}

float readAcc(byte numAcc, byte axis) {
  int highdata, lowdata;
  int highaddr, lowaddr;
  int pin;
  switch(numAcc) {
    case 0:
      pin = CS1;
      break;
    case 1:
      pin = CS2;
      break;
    case 2:
      pin = CS3;
      break; 
    case 3:
      pin = CS4;
      break;
    default:
      return 0;
  }
  switch(axis) {
    case XAXIS:
      highaddr = 0x33;
      lowaddr = 0x32;
      break;
    case YAXIS:
      highaddr = 0x35;
      lowaddr = 0x34;
      break;
    case ZAXIS:
      highaddr = 0x37;
      lowaddr = 0x36;
      break;
    default:
      return 0;
  }
  highaddr |= (1 << 7) | (0 << 6); // read, single transaction
  highaddr = highaddr << 8;
  lowaddr |= (1 << 7) | (0 << 6); // read, single transaction
  lowaddr = lowaddr << 8;
  
  SPI.beginTransaction(hockiSPISettings);
  digitalWrite(pin, LOW);
  highdata = SPI.transfer16(highaddr);
  digitalWrite(pin, HIGH);
  SPI.endTransaction();
  
  SPI.beginTransaction(hockiSPISettings);
  digitalWrite(pin, LOW);
  lowdata = SPI.transfer16(lowaddr);
  digitalWrite(pin, HIGH);
  SPI.endTransaction();

  highdata &= 0x00FF;
  lowdata &= 0x00FF;

  int alldata = (highdata << 8) | lowdata;
  float ret = alldata * ACCSF;
  return ret;
}
