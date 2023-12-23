#pragma once

#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include "Sensors/Sensors.h"
using namespace BLA;

class KalmanFilter {
	public:
		struct quaternion {
			float w;          //real value
			float x, y, z;    //imaginary values
		};
		KalmanFilter(Sensors &sens, float latitude);
		void init();
		void filter(float dT);
		Matrix<4> x;

	private:
		Sensors &sens;
		Matrix<4> x_prior;
		Matrix<4,4> F;
		Matrix<3> w;
		Matrix<4,4> Q; //process noise covariance
		Matrix<4,4> P; //state covariance
		Matrix<4,4> P_prior; //previous state covariance estimate
		Matrix<3> m; //magnetometer
		Matrix<3> mBase; //magnetometer base vector
		Matrix<3,3> R;	//measurement noise covariance
		Matrix<3> z; //measurement
		Matrix<3> h; //measurement model
		Matrix<3,4> H; //measurement model jacobian
		Matrix<4,4> I4; //the 4x4 identity matrix
		Matrix<3> v; //innovation
		Matrix<3,3> S; //innovation covariance
		Matrix<4,3> K; //kalman gain		
		Matrix<3> rotate(quaternion q, Matrix<3> w);
		void predict(float dT);
		void update();
		float dipAngle;

};