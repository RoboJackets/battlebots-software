#include "src/Eigen/Eigen337.h"
#include "src/Eigen/Dense.h"
#include "src/HEKF/HEKF.h"
#include "SPI.h"
using namespace Eigen;

// DEFINE HEKF PARAMS //
#define HEKFDT 1e-4
#define ALPHA  


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

// DEFINE PINS - MAGS //
#define MAGA 38
#define MAGB 39
#define SRPIN 32

// DEFINE PINS - IR RECVS //
#define INT10K 23
#define INT20K 22
#define PK10K 20
#define PK20K 21
#define IRSEL 37

SPISettings hockiSPISettings(SPIRATE, MSBFIRST, SPI_MODE3);

unsigned long t0;
unsigned long t1;
HEKF* filter;
unsigned long microControlPeriod = 1000;

void setup() {

  // set up chip selects
  pinMode(CS1, OUTPUT); digitalWrite(CS1, HIGH);
  pinMode(CS2, OUTPUT); digitalWrite(CS2, HIGH);
  pinMode(CS3, OUTPUT); digitalWrite(CS3, HIGH);
  pinMode(CS4, OUTPUT); digitalWrite(CS4, HIGH);

  // set up SPI
  SPI.begin();
  SPI.setMOSI(MOSIPIN);
  SPI.setMISO(MISOPIN);
  SPI.setSCK(SCLKPIN);
  //  SPI.usingInterrupt();

  // set up HEKF Filter
  *filter = 


}

//void readAcc(

void loop() {
  t0 = micros();

  // control algorithm goes here

  t1 = micros();
  delayMicroseconds(microControlPeriod - (t1 - t0)); 
  // lol this will fucking break if the shit above takes longer that 1 millisecond
  
}
