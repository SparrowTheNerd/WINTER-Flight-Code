#pragma once

#include <Arduino.h>
#include "Sensors/Sensors.h"
#include <eigen-3.4.0/Eigen/Dense>
#include <eigen-3.4.0/Eigen/Geometry>
#include <eigen-3.4.0/Eigen/LU>

using Eigen::Matrix;
using Eigen::Vector;

class KalmanFilter {
	public:
		struct quaternion {
			float w;          //real value
			float x, y, z;    //imaginary values
		};
		KalmanFilter(Sensors &sens);
		void init(float stddev_mg, float stddev_ps);
		void filter(float dt);
		Vector<float,10> x; //x_n,n

	private:
		Vector<float,4> uXl; // cntrl vector for accel
		Vector<float,3> uGy; // cntrl vector for gyros
		Vector<float,4> zMg; // obs vector for magns
		Vector<float,1> zBm; // obs vector for press
		Vector<float,10> x_prior; //x_n,n-1
		Matrix<float,4,10> Hmg;  //mag obs matrix
		Matrix<float,1,10> Hbm;  //baro obs matrix
		Matrix<float,10,4> G;    //cntrl matrix
		Matrix<float,10,10> F;   //state transn matrix
		Matrix<float,10,10> P;   //covariance matrix
		Matrix<float,10,10> P_prior;  //covariance prior prediction
		Matrix<float,4,4> Rmg;   //mag meas covar matrix
		Matrix<float,1,1> Rbm;   //baro meas covar matrix
		Matrix<float,10,4> Kmg;  //mag kalman gain
		Vector<float,10>   Kbm;  //baro kalman gain

		LSM9DS1 IMU;
		Vector<float,3> mgBase;  //initial magnetometer vector, for comparing against

		Sensors& sens;

		void gain();
		void update(bool magAvail, bool baroAvail);
		void extrapolate(float dt);
		void normalize();
		Vector<float,3> measTransform(quaternion q, Vector<float,3> meas);
		int normCounter=0;

};