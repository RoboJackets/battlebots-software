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

const unsigned int SENSOR_TYPES = 4;
constexpr unsigned int SENSOR_COUNTS[SENSOR_TYPES] = {6,		1,		1,		1};
constexpr float SENSOR_VARS[SENSOR_TYPES] = 		 {80,		0.001,	30,		30};
constexpr float SENSOR_ERROR_LIMIT[SENSOR_TYPES] =   {2,		1,		10,		10};
constexpr float SENSOR_ERROR_H[SENSOR_TYPES] = 	     {3000,		3000,	3000,	3000};
constexpr float SENSOR_ERROR_L[SENSOR_TYPES] =       {100,		100,	100,	100};
constexpr float ACC_DISTS[SENSOR_COUNTS[ACC]] = 	 {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};

typedef struct sensorData {
	float magDir;
	unsigned int* errCount[SENSOR_TYPES];
} SensorData;

//Function to initialize sensor data
SensorData* initSensorData();

//Function to destroy sensor data
void destroySensorData(SensorData* sData);

#endif
