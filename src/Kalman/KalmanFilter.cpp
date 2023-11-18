#include "KalmanFilter.h"
using namespace BLA;

KalmanFilter::KalmanFilter(Sensors& sensor) : sens(sensor) {};

void KalmanFilter::init() {
	x = {1.f, 0.f, 0.f, 0.f};
}

void KalmanFilter::filter(float dT) {
	sens.getData();
	Matrix<3> gy = {sens.gX, sens.gY, sens.gZ};
	quaternion q = {x(0), x(1), x(2), x(3)};
	w = rotate(q,gy);
	float X = w(0)*dT/2.f; float Y = w(1)*dT/2.f; float Z = w(2)*dT/2.f;
	F = {1.f,  -X, -Y , -Z ,
				 X, 1.f,  Z , -Y ,
				 Y, -Z , 1.f,  X ,
				 Z,  Y,  -X , 1.f};
	Matrix<4> x_new = F*x;
	x = x_new;
	float xMag = sqrtf(x(0)*x(0)+x(1)*x(1)+x(2)*x(2)+x(3)*x(3));
	x /= xMag;	//normalize
}

Matrix<3> KalmanFilter::rotate(quaternion q, Matrix<3> meas) {
	Matrix<3,3> R = { q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z, 2*(q.x*q.y + q.w*q.z), 2*(q.x*q.z - q.w*q.y),
										2*(q.x*q.y - q.w*q.z), q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z, 2*(q.y*q.z + q.w*q.x),
										2*(q.x*q.z + q.w*q.y), 2*(q.y*q.z - q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z };
	return R*meas;
}