#ifndef HEKF_H
#define HEKF_H

#include <math.h>
#include "../Eigen/Eigen337.h"
#include "../Eigen/Dense.h"
#include "Sensors.h"
using namespace Eigen;

#define ANGLE_VAR 0.1	//Variance of the initial angle
#define PROCESS_VAR 0.1	//Variance of dynamics model

//Pointer to function accepting void, returning matrix
typedef MatrixXf (*void_matFunc) (void);

//HEKF Structure
typedef struct hekf {
	float dt;						//Time step for CT simulation
	unsigned long tUpdate;			//Time of last update (in us)

	Matrix<float, 2, 2> A;			//Motion matrix
	Matrix<float, 2, 1> B;			//Input matrix
	Matrix<float, 2, 1> x;			//Current state estimate
	float u;						//Current input to the system

	void_matFunc hTable[SENSORS];	//Array of state to observation matricies function pointers
	void_matFunc HTable[SENSORS];	//Array of state to observation Jacobians function pointers

	Matrix<float, 2, 2> P;			//State covariance
	Matrix<float, 2, 2> Q;			//Process noise covariance
} HEKF;

//Pointer to function accepting pointer to HEKF and matrix, returning matrix
typedef MatrixXf (*mat_matFunc) (HEKF*, MatrixXf);

//Function to initialize HEKF
HEKF* initHEKF(float dt, float alpha, float beta, float wheelR, float botR);

//Function to update HEKF
unsigned int updateHEKF(HEKF* filter, Matrix<float, Dynamic, 1> meas, 
	float input, unsigned long t, Sensor sensor);

//Function to constrain a number to the range (-pi, pi)
inline float wrapToPi(float x) {
	return remainder(x, 2 * M_PI);
}

//Function to free HEKF pointer
void destroyHEKF(HEKF* filter);

#endif
