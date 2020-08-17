#include "../Eigen/Eigen337.h"
#include "../Eigen/Dense.h"
#include "HEKF.h"
#include "Sensors.h"
using namespace Eigen;

SensorData* initSensorData() {
	SensorData* sData = (SensorData*)malloc(sizeof(SensorData));
	if(sData == NULL) {
		return sData;
	}
	sData->magDir = 0;
	for(int i = 0; i < SENSOR_TYPES; i++) {
		sData->errCount[i] = (unsigned int*)calloc(SENSOR_COUNTS[i], sizeof(unsigned int));
	}
	return sData;
}

void destroySensorData(SensorData* sData) {
	if(sData != NULL) {
		free(sData->errCount);
	}
	free(sData);
}
