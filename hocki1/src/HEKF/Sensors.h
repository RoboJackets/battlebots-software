#ifndef SENSORS_H
#define SENSORS_H

#include "../Eigen/Eigen337.h"
#include "../Eigen/Dense.h"
using namespace Eigen;

//Enum containing sensor types
enum Sensor {
	ACC,
	BEACON,
	MAG_Y,
	MAG_X
};

#define ACC_VAR 80			//Variance of the accelerometer
#define BEACON_VAR 0.001	//Variance of the beacon
#define MAG_VAR 30			//Variance of the magnetometer
#define MAG_DIR 0			//Offset of magnetometer
#define SENSORS 4			//Total unique types of sensors used
//Number of each sensor, in order of: accelerometer pairs, beacon, magY, magX
constexpr unsigned int SENSOR_COUNTS[SENSORS] = {6, 1, 1, 1};
//The distance of each accelerometer pair
const float ACC_DISTS[SENSOR_COUNTS[ACC]] = {0.0257, 0.0363, 0.0257, 0.0257, 0.0363, 0.0257};

/*
	Functions returning the state to observation matricies of each sensor
	Parameters:
		x: current state
		BMag: magnitude of total B-field
	Return: state to observation matrix
*/
MatrixXf acc_h (Matrix<float, 2, 1> x);
MatrixXf beacon_h (Matrix<float, 2, 1> x);
MatrixXf magy_h (Matrix<float, 2, 1> x, float BMag);
MatrixXf magx_h (Matrix<float, 2, 1> x, float BMag);

/*
	Functions returning the state to observation jacobian of each sensor
	Parameters:
		x: current state
		BMag: magnitude of total B-field
	Return: state to observation jacobian
*/
MatrixXf acc_H (Matrix<float, 2, 1> x);
MatrixXf beacon_H (Matrix<float, 2, 1> x);
MatrixXf magy_H (Matrix<float, 2, 1> x, float BMag);
MatrixXf magx_H (Matrix<float, 2, 1> x, float BMag);

/*
	Function returning the sensor noise covariance for each sensor
	Parameters:
		s: Sensor to get
	Return: noise covariance
*/
MatrixXf getR (Sensor s);

#endif
