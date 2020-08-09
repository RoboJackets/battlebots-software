#include "../Eigen/Eigen337.h"
#include "../Eigen/Dense.h"
#include "HEKF.h"
#include "Sensors.h"
#include <limits>
using namespace Eigen;


/*
	Intializes an HEKF
	Parameters:
		dt: Time step for CT simulation
		alpha: motor paramter
		beta: motor parameter
		wheelR: radius of the wheel
		botR: radius from bot center to wheel location
	Return: Pointer to the initialized HEKF

	NOTE: The caller of this function is responsible for verifying the
	returned pointer is not NULL
*/
HEKF* initHEKF(float dt, float alpha, float beta, float wheelR, float botR) {
	HEKF* filter = (HEKF*) malloc(sizeof(HEKF));		//Allocate space
	if (filter == NULL) {								//Handle malloc failure
		return filter;
	}
	filter->dt = dt;								
	filter->tUpdate = 0;
	filter->A << 0, 1, 
			     0, -alpha;
    filter->B << 0, 
                 wheelR * beta / botR;
    filter->x << 0, 0;									//Initial state estimate
    filter->u = 0;										//Initial input to system

    //Initialize hTable function pointers
    void_matFunc hInitializer[SENSORS] = {
    	(void_matFunc)&acc_h, 
    	(void_matFunc)&beacon_h,
		(void_matFunc)&magy_h,
		(void_matFunc)&magx_h
    };
    memcpy(filter->hTable, hInitializer, sizeof(hInitializer));

    //Initialize HTable function pointers
    void_matFunc HInitializer[SENSORS] = {
    	(void_matFunc)&acc_H,
    	(void_matFunc)&beacon_H,
		(void_matFunc)&magy_H,
		(void_matFunc)&magx_H
    };
    memcpy(filter->HTable, HInitializer, sizeof(HInitializer));	

    //Initialize RTable Values
    filter->RTable[0] = MatrixXf::Identity(SENSOR_COUNTS[ACC], SENSOR_COUNTS[ACC]) * ACC_VAR;
    filter->RTable[1] = MatrixXf::Constant(SENSOR_COUNTS[BEACON], 1, BEACON_VAR);
    filter->RTable[2] = MatrixXf::Constant(SENSOR_COUNTS[MAG_Y], 1, MAG_VAR);
    filter->RTable[3] = MatrixXf::Constant(SENSOR_COUNTS[MAG_X], 1, MAG_VAR);

    filter->P << ANGLE_VAR, 0,							//Covariance of angular velocity is 0
    			 0, 0;
   	filter->Q << PROCESS_VAR, 0,
   	             0, PROCESS_VAR;
   	return filter;
}

/*
	Evaluates the derivative of state
	Parameters:
		filter: pointer to an HEKF
		x: state at which to evaluate derivative
	Return: derivative of state at x
*/
MatrixXf xDot(HEKF* filter, MatrixXf x) {
	return filter->A * x + filter->B * filter->u;
}

/*
	Evaluates the derivative of state covariance
	Parameters:
		filter: pointer to an HEKF
		P: covariance at which to evaluate derivative
	Return: derivative of state covariance at P
*/
MatrixXf PDot(HEKF* filter, MatrixXf P) {
	return filter->A * P + P * (filter->A).transpose() + filter->Q;
}

/*
	Integrates the given matrix for a set time
	Parameters:
		filter: pointer to an HEKF
		x: initial matrix value
		dx: function pointer to derivative function
		T: time (in seconds) over which to integrate
	Return: the final matrix value after integration
*/
MatrixXf integrate(HEKF* filter, MatrixXf x, mat_matFunc dx, float T) {
	unsigned int steps = (unsigned int) (T / filter-> dt);
	for (unsigned int i = 0; i < steps; i++) {
		x = x + dx(filter, x) * filter->dt;
	}
	return x;
}

/*
	Updates the HEKF
	Parameters:
		filter: pointer to an HEKF
		meas: measurement vector. Meas is an N x 1 matrix where N is the
			number of sensors available for the type being updated
			In the event that a magnetometer is used for update, the last
			element of meas should hold the BMag term (so meas becomes a (N+1) x 1 matrix)
		input: input to the system
		t: time (in us) of the update
		sensor: the type of sensor to execute update with
	Return: whether the update successfully completed (1: successful, 0: failure)
*/
unsigned int updateHEKF(HEKF* filter, Matrix<float, Dynamic, 1> meas, 
	float input, unsigned long t, Sensor sensor) {
	//Check to make sure measurement input is correct
	if((unsigned int)meas.rows() != SENSOR_COUNTS[sensor]) {
		return 0;
	}

	//Prediction step
	float T = (float)(t - filter->tUpdate) / 1e6;		//Time since last update in seconds
	filter->u = input;									//Set the current input
	filter->x = integrate(filter, filter->x, &xDot, T);	//Integrate state
	filter->P = integrate(filter, filter->P, &PDot, T);	//Integrate state covariance

	//Update step
	MatrixXf hk;		//Value of expected measurement for given state
	MatrixXf Hk;		//Jacobian of hk
	switch(sensor) {	//Evaluate h/Hk for the selected sensor
		case(ACC):
		case(BEACON):	
			hk = ((MatrixXf (*)(Matrix<float, 2, 1>))	//cast generic function pointer to specific function
				(filter->hTable[sensor]))(filter->x);	//pointer based on the selected sensor and evaluate
			Hk = ((MatrixXf (*)(Matrix<float, 2, 1>))
				(filter->HTable[sensor]))(filter->x);
			break;
		case(MAG_Y):
		case(MAG_X):
			float BMag = meas(SENSOR_COUNTS[MAG_X]);
			hk = ((MatrixXf (*)(Matrix<float, 2, 1>, float))
				(filter->hTable[sensor]))(filter->x, BMag);
			Hk = ((MatrixXf (*)(Matrix<float, 2, 1>, float))
				(filter->HTable[sensor]))(filter->x, BMag);
			break;
	}
	MatrixXf Rk = filter->RTable[sensor];					//Select corresponding sensor covariances						
	Matrix<float, 2, 2> K = filter->P * Hk.transpose() * 	//Calculate kalman gain
		(Hk * filter->P * Hk.transpose() + Rk).inverse();
	MatrixXf err = meas - hk;								//Difference between true measurement and expected measurement
	if(sensor == BEACON) {									//If using beacon, error should be wrapped to (-pi, pi)
		err(0) = wrapToPi(err(0));
	}
	filter->x = filter->x + K * err;								//Update state
	filter->P = (MatrixXf::Identity(2, 2) - K * Hk) * filter->P * 	//Update covariance
				(MatrixXf::Identity(2, 2) - K * Hk).transpose() + K * Rk * K.transpose();

	//Update values		
	(filter->x)(0) = wrapToPi((filter->x)(0));	//Constrain position to range (-pi, pi)
	filter->tUpdate = t;						//Set time of last update to current time
	return 1;
}

/*
	Frees the memory allocated to the passed HEKF pointer
	Parameters:
		filter: pointer to the HEKF to free
*/
void destroyHEKF(HEKF* filter) {
	if(filter != NULL) {
		free(filter->hTable);
		free(filter->HTable);
	}
	free(filter);
}

/*
	Kills a sensor
	Parameters:
		filter: pointer to the HEKF
		sensor: sensor type to kill
		idx: index of the sensor type to kill
	Return: whether the sensor was successfully killed (1: successful, 0: failure)
*/
unsigned int killSensor(HEKF* filter, Sensor sensor, unsigned int idx) {
	if(idx >= SENSOR_COUNTS[sensor]) {
		return 0;
	}
	(filter->RTable[sensor])(idx, idx) = std::numeric_limits<float>::max();
	return 1;
}

/*
	Revives a sensor
	Parameters:
		filter: pointer to the HEKF
		sensor: sensor type to revive
		idx: index of the sensor type to revive
	Return: whether the sensor was successfully killed (1: successful, 0: failure)
*/
unsigned int reviveSensor(HEKF* filter, Sensor sensor, unsigned int idx) {
	if(idx >= SENSOR_COUNTS[sensor]) {
		return 0;
	}
	float covariance;
	switch(sensor) {
		case ACC:
			covariance = ACC_VAR;
			break;
		case BEACON:
			covariance = BEACON_VAR;
			break;
		case MAG_Y:
		case MAG_X:
			covariance = MAG_VAR;
			break;
	}
	(filter->RTable[sensor])(idx, idx) = covariance;
	return 1;
}