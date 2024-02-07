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
		Vector<float,10> x;
		Matrix<float,10,10> F;

	private:
		Sensors &sens;
		Vector<float,10> x_pred;	//predicted state estimate
		
		Vector3f w;	//angular velocity in rads/s
		Matrix<float,10,6> W; //process noise jacobian
		Matrix<float,10,10> Q; //process noise covariance
		Matrix<float,10,10> P; //state covariance
		Matrix<float,10,10> P_pred; //predicted state covariance
		Vector3f m; //magnetometer
		Vector3f mBase; //magnetometer base vector
		Vector3f mInit; //initial magnetometer vector
		Vector3f aBase; //accelerometer base vector
		Vector3f aInit; //initial accelerometer vector
		Matrix3f R;	//measurement noise covariance
		Vector3f z; //measurement
		Vector3f h; //measurement model
		Matrix<float,3,10> H; //measurement model jacobian
		Matrix<float,10,10> I10; //the 10x10 identity matrix
		Vector3f v; //innovation
		Matrix3f S; //innovation covariance
		Matrix<float,10,3> K; //kalman gain		
		Vector3f rotate(Vector<float,10> A, Vector3f w);
		Vector3f invRotate(Vector<float,10> A, Vector3f w);
		int sign(float x);
		void predict(float dT);
		void correct();
		float dipAngle;
};