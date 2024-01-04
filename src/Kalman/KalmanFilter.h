#pragma once

#include <Arduino.h>
//#include <BasicLinearAlgebra.h>
#include "Sensors/Sensors.h"
#include <ArduinoEigenDense.h>
using namespace Eigen;
//using namespace BLA;

class KalmanFilter {
	public:
		struct quaternion {
			float w;          //real value
			float x, y, z;    //imaginary values
		};
		KalmanFilter(Sensors &sens, float latitude);
		void init();
		void filter(float dT);
		Vector<float,4> x;
		Matrix4f F;

	private:
		Sensors &sens;
		Vector<float,4> x_pred;	//predicted state estimate
		
		Vector3f w;	//angular velocity in rads/s
		Matrix4f W; //process noise jacobian
		Matrix4f Q; //process noise covariance
		Matrix4f P; //state covariance
		Matrix4f P_pred; //predicted state covariance
		Vector3f m; //magnetometer
		Vector3f mBase; //magnetometer base vector
		Matrix3f R;	//measurement noise covariance
		Vector3f z; //measurement
		Vector3f h; //measurement model
		Matrix<float,3,4> H; //measurement model jacobian
		Matrix4f I4; //the 4x4 identity matrix
		Vector3f v; //innovation
		Matrix3f S; //innovation covariance
		Matrix<float,4,3> K; //kalman gain		
		Vector3f rotate(Vector<float,4> A, Vector3f w);
		void predict(float dT);
		void correct();
		float dipAngle;
};