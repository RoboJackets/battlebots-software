#include <math.h>
#include "src/Eigen/Eigen337.h"
#include "src/Eigen/Dense.h"
#include "Sensors.h"
using namespace Eigen;

//These functions calculate various sensor-related values
//See Sensors.h for further description

MatrixXf acc_h (Matrix<float, 2, 1> x) {
	Matrix<float, SENSOR_COUNTS[ACC], 1> h;
	for(unsigned int i = 0; i < SENSOR_COUNTS[ACC]; i++) {
		h(i) = pow(x(1), 2) * ACC_DISTS[i];
	}
	return h;
}

MatrixXf beacon_h (Matrix<float, 2, 1> x) {
	Matrix<float, SENSOR_COUNTS[BEACON], 1> h;
	for(unsigned int i = 0; i < SENSOR_COUNTS[BEACON]; i++) {
		h(i) = x(0);
	}
	return h;
}

MatrixXf magy_h (Matrix<float, 2, 1> x, float BMag) {
	Matrix<float, SENSOR_COUNTS[MAG_Y], 1> h;
	for(unsigned int i = 0; i < SENSOR_COUNTS[MAG_Y]; i++) {
		h(i) = BMag * cos(x(0) - MAG_DIR);
	}
	return h;
}

MatrixXf magx_h (Matrix<float, 2, 1> x, float BMag) {
	Matrix<float, SENSOR_COUNTS[MAG_X], 1> h;
	for(unsigned int i = 0; i < SENSOR_COUNTS[MAG_X]; i++) {
		h(i) = BMag * -sin(x(0) - MAG_DIR);
	}
	return h;
}

MatrixXf acc_H (Matrix<float, 2, 1> x) {
	Matrix<float, SENSOR_COUNTS[ACC], 2> H;
	for(unsigned int i = 0; i < SENSOR_COUNTS[ACC]; i ++) {
		H(i, 0) = 0;
		H(i, 1) = 2 * x(1) * ACC_DISTS[i];
	}
	return H;
}

MatrixXf beacon_H (Matrix<float, 2, 1> x) {
	Matrix<float, SENSOR_COUNTS[BEACON], 2> H;
	for(unsigned int i = 0; i < SENSOR_COUNTS[BEACON]; i++) {
		H(i, 0) = 1;
		H(i, 1) = 0;
	}
	return H;
}

MatrixXf magy_H (Matrix<float, 2, 1> x, float BMag) {
	Matrix<float, SENSOR_COUNTS[MAG_Y], 2> H;
	for(unsigned int i = 0; i < SENSOR_COUNTS[MAG_Y]; i++) {
		H(i, 0) = BMag * -sin(x(0) - MAG_DIR);
		H(i, 1) = 0;
	}
	return H;
}

MatrixXf magx_H (Matrix<float, 2, 1> x, float BMag) {
	Matrix<float, SENSOR_COUNTS[MAG_X], 2> H;
	for(unsigned int i = 0; i < SENSOR_COUNTS[MAG_X]; i++) {
		H(i, 0) = BMag * -cos(x(0) - MAG_DIR);
		H(i, 1) = 0;
	}
	return H;
}

MatrixXf getR (Sensor s) {
	switch(s) {
		case(ACC):
			return MatrixXf::Identity(SENSOR_COUNTS[ACC], SENSOR_COUNTS[ACC]) * ACC_VAR;
		case(BEACON):
			return MatrixXf::Constant(SENSOR_COUNTS[BEACON], 1, BEACON_VAR);
		case(MAG_Y):
			return MatrixXf::Constant(SENSOR_COUNTS[MAG_Y], 1, MAG_VAR);
		case(MAG_X):
			return MatrixXf::Constant(SENSOR_COUNTS[MAG_X], 1, MAG_VAR);
	}
}