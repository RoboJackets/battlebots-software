#include "src/Eigen/Eigen337.h"
#include "src/Eigen/Dense.h"
using namespace Eigen;

//Declare benchmarking parameters
unsigned int tgtFreq, duration, updates, matMults, vecMults, invs;
float dt;
void setup() {
  Serial.begin(115200);
  tgtFreq = 1000; // Target update frequency
  dt = 1e-4; // CT timestep
  duration = 5; // Expected time to simulate

  updates = duration * tgtFreq; // Number of updates to execute

  matMults = 10;
  vecMults = 4 * ((1 / (float)tgtFreq) / dt); // number of matrix by vector multiplications per update
  invs = 1; // number of matrix inversions per update
}

void loop() { 
  //Dummy matricies to operate on
  Matrix<float, 2, 2> A;
  A << 0.1, 0.2, 0.3, 0.4;
  Matrix<float, 2, 2> B;
  B << 0.1, 0.2, 0.3, 0.4;
  Matrix<float, 2, 1> C;
  C << 0.1, 0.2;
  
  Matrix<float, 2, 2>* temp2_2 = (Matrix<float, 2, 2>*) malloc(sizeof(Matrix<float, 2, 2>));
  Matrix<float, 2, 1>* temp2_1 = (Matrix<float, 2, 1>*) malloc(sizeof(Matrix<float, 2, 1>));

  unsigned long startT = micros(); // Start time

  for(unsigned int i = 0; i < updates; i++) {
    for(unsigned int n = 0; n < matMults; n++) {
      *temp2_2 = A * B;
    }
    for(unsigned int n = 0; n < vecMults; n++) {
      *temp2_1 = A * C;
    }
    for(unsigned int n = 0; n < invs; n++) {
      *temp2_2 = A.inverse();
    }
  }
  free(temp2_2);
  free(temp2_1);

  unsigned long endT = micros(); // End time
  unsigned int diff = (float)(endT - startT) / 1e6; // Difference in seconds
  unsigned int trueFreq = updates / diff;

  char result[100];
  sprintf(result, "Time difference: %d s", diff);
  Serial.println(result);
  sprintf(result, "True frequency: %d Hz", trueFreq);
  Serial.println(result);
  delay(5000);
}
