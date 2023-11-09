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
		KalmanFilter(Sensors &sens);
		void init(float stddev_mg, float stddev_ps);
		void filter(float dt);
		Matrix<10> x; //x_n,n
		

	private:
		Matrix<4> uXl; // cntrl vector for accel
		Matrix<3> uGy; // cntrl vector for gyros
		Matrix<4> zMg; // obs vector for magns
		Matrix<1> zBm; // obs vector for press
		Matrix<10> x_prior; //x_n,n-1
		Matrix<4,10> Hmg;  //mag obs matrix
		Matrix<1,10> Hbm;  //baro obs matrix
		Matrix<10,4> G;    //cntrl matrix
		Matrix<10,10> F;   //state transn matrix
		Matrix<10,10> P;   //covariance matrix
		Matrix<10,10> P_prior;  //covariance prior prediction
		Matrix<4,4> Rmg;   //mag meas covar matrix
		Matrix<1,1> Rbm;   //baro meas covar matrix
		Matrix<10,4> Kmg;  //mag kalman gain
		Matrix<10>   Kbm;  //baro kalman gain

		LSM9DS1 IMU;
		Matrix<3> mgBase;  //initial magnetometer vector, for comparing against

		Sensors& sens;

		void gain();
		void update();
		void extrapolate(float dt);
		void normalize();
		Matrix<3> crossProd(Matrix<3> A, Matrix<3> B);
		Matrix<3> measTransform(quaternion q, Matrix<3> meas);
		int normCounter=0;

};