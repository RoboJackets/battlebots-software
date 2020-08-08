#include <iostream>
#include "../Eigen/Eigen337.h"
#include "../Eigen/Dense.h"
#include "HEKF.h"
#include "Sensors.h"
using namespace Eigen;

int main() {
	HEKF* filter = initHEKF(1e-6, 2, 4, 1.5, 5.5);
	killSensor(filter, ACC, 1);
	reviveSensor(filter, ACC, 1);
	std::cout << filter->RTable[0] << std::endl;

	// Matrix<float, 6, 1> meas;
	// meas << 2, 2, 2, 2, 2, 2;
	// for(unsigned int i = 1; i <= 10; i++) {
	// 	if(updateHEKF(filter, meas, 6, 100000 * i, ACC) == 1) {
	// 		std::cout << filter->x << std::endl;
	// 	} else {
	// 		std::cout << "Error: Sensor and measurement mismatch" << std::endl;
	// 	}
	// }
}
