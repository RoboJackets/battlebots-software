#ifndef HEKF_H
#define HEKF_H

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
	float dt;							//Time step for CT simulation
	unsigned long tUpdate;				//Time of last update (in us)

	Matrix<float, 2, 2> A;				//Motion matrix
	Matrix<float, 2, 1> B;				//Input matrix
	Matrix<float, 2, 1> x;				//Current state estimate
	float u;							//Current input to the system

	void_matFunc hTable[SENSOR_TYPES];	//Array of state to observation matricies function pointers
	void_matFunc HTable[SENSOR_TYPES];	//Array of state to observation Jacobians function pointers
	MatrixXf RTable[SENSOR_TYPES];		//Array of sensor covariance matricies
	SensorData* sData;					//Variable sensor data (constant sensor data stored in static memory)

	Matrix<float, 2, 2> P;				//State covariance
	Matrix<float, 2, 2> Q;				//Process noise covariance
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

//Function to kill a malfunctioning sensor
unsigned int killSensor(HEKF* filter, Sensor sensor, unsigned int idx);

//Function to revive a sensor
unsigned int reviveSensor(HEKF* filter, Sensor sensor, unsigned int idx);

//Function to update sensor error counters
void updateErrCounts(HEKF* filter, MatrixXf error, Sensor sensor);

/*
	Functions returning the state to observation matricies of each sensor
	Parameters:
		x: current state
		BMag: magnitude of total B-field
	Return: state to observation matrix
*/
MatrixXf acc_h (HEKF* filter);
MatrixXf beacon_h (HEKF* filter);
MatrixXf magy_h (HEKF* filter, float BMag);
MatrixXf magx_h (HEKF* filter, float BMag);

/*
	Functions returning the state to observation jacobian of each sensor
	Parameters:
		x: current state
		BMag: magnitude of total B-field
	Return: state to observation jacobian
*/
MatrixXf acc_H (HEKF* filter);
MatrixXf beacon_H (HEKF* filter);
MatrixXf magy_H (HEKF* filter, float BMag);
MatrixXf magx_H (HEKF* filter, float BMag);

#endif
