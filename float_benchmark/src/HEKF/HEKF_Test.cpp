#include <iostream>
#include "../Eigen/Eigen337.h"
#include "../Eigen/Dense.h"
#include "HEKF.h"
#include "Sensors.h"
using namespace Eigen;

int main() {
	HEKF* filter = initHEKF(1e-6, 2, 4, 1.5, 5.5);
	Matrix<float, 6, 1> meas;
	meas << 20000, 2, 2, 2, 2, 2;
	for(unsigned int i = 1; i <= 100; i++) {
		if(updateHEKF(filter, meas, 6, 100000 * i, ACC) == 1) {
			std::cout << filter->x << "\n" << std::endl;
		} else {
			std::cout << "Error: Sensor and measurement mismatch" << std::endl;
		}
	}
}
