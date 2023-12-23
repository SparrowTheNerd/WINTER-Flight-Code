#include "KalmanFilter.h"
using namespace BLA;

float sigGy = 0.3f; //spectral density noise thingy (NEEDS MORE RESEARCH)
float sigMg = 0.001f;

KalmanFilter::KalmanFilter(Sensors& sensor, float latitude) : sens(sensor) {
  this->dipAngle = atanf(latitude*DEG_TO_RAD);
  this->I4 = {1.f, 0.f, 0.f, 0.f,
              0.f, 1.f, 0.f, 0.f,
              0.f, 0.f, 1.f, 0.f,
              0.f, 0.f, 0.f, 1.f};
};

void KalmanFilter::init() { //set initial values for filter
	x = {1.f, 0.f, 0.f, 0.f};
  
  R = {sigMg, 0.f, 0.f,
         0.f, sigMg, 0.f,
         0.f, 0.f, sigMg};

  while(!sens.IMU.magAvailable()) {  //wait for magnetometer data
    delay(1);
  }
  sens.getData();
  mBase = {sens.mX, sens.mY, sens.mZ};
  sens.prev_filtered_mX = sens.mX; sens.prev_filtered_mY = sens.mY; sens.prev_filtered_mZ = sens.mZ; sens.prev_mX = sens.mX; sens.prev_mY = sens.mY; sens.prev_mZ = sens.mZ;
  sens.magFilter = true;
  P = I4;
}

void KalmanFilter::predict(float dT) {
	Matrix<3> gy = {sens.gX, sens.gY, sens.gZ};
  quaternion q = {x(0), x(1), x(2), x(3)};

  w = gy;
	float X = w(0)*dT/2.f; float Y = w(1)*dT/2.f; float Z = -w(2)*dT/2.f;
  //X=0; Y=0; Z=0;
	F = {1.f,  -X, -Y , -Z ,    //jacobian wrt quaternion
				 X, 1.f,  Z , -Y ,
				 Y, -Z , 1.f,  X ,
				 Z,  Y,  -X , 1.f};

	x_prior = F*x; //state prediction
	x_prior /= sqrtf(x_prior(0)*x_prior(0)+x_prior(1)*x_prior(1)+x_prior(2)*x_prior(2)+x_prior(3)*x_prior(3));	//normalize

  Matrix<4,3> W = {-x(1), -x(2), -x(3),     //jacobian wrt angular rate
                    x(0), -x(3),  x(2),
                    x(3),  x(0), -x(1),
                   -x(2),  x(1),  x(0)};
  W *= dT/2.f;
  Q = W*~W*sigGy;
  P_prior = F * P * ~F + Q;
}

void KalmanFilter::update() {  
  quaternion q = {x_prior(0), x_prior(1), x_prior(2), x_prior(3)};  //predicted quaternion

  // Expected magnetometer reading
  Matrix<3> m_expected = rotate(q, mBase);

  // Actual magnetometer reading
  m = Matrix<3> {sens.mX, sens.mY, sens.mZ};
  z = m; //measurement
  h = m_expected; //measurement model

  // Jacobian of the measurement model
  H = {mBase(0)*q.w+mBase(1)*q.z-mBase(2)*q.y, mBase(0)*q.x+mBase(1)*q.y+mBase(2)*q.z, -mBase(0)*q.y+mBase(1)*q.x-mBase(2)*q.w, -mBase(0)*q.z+mBase(1)*q.w+mBase(2)*q.x,
      -mBase(0)*q.z+mBase(1)*q.w+mBase(2)*q.x, mBase(0)*q.y-mBase(1)*q.x+mBase(2)*q.w,  mBase(0)*q.x+mBase(1)*q.y+mBase(2)*q.z, -mBase(0)*q.w-mBase(1)*q.z+mBase(2)*q.y,
       mBase(0)*q.y-mBase(1)*q.x+mBase(2)*q.w, mBase(0)*q.z-mBase(1)*q.w-mBase(2)*q.x,  mBase(0)*q.w+mBase(1)*q.z-mBase(2)*q.y,  mBase(0)*q.x+mBase(1)*q.y+mBase(2)*q.z};
  H *= 2.f;
  Matrix<4,3> Ht = ~H;
  v = z - h;
  S = H * P_prior * Ht + R;
  K = P_prior * Ht * Inverse(S);

  // Serial << m << m_expected;

  // Update state estimate
  x = x_prior + K * v;
  float xMag = sqrtf(x(0)*x(0)+x(1)*x(1)+x(2)*x(2)+x(3)*x(3));
	x /= xMag;	//normalize

  // Update state covariance
  P = (I4 - K * H) * P_prior;
}

void KalmanFilter::filter(float dT) {
  predict(dT);
  update();
}

Matrix<3> KalmanFilter::rotate(quaternion q, Matrix<3> meas) {
  // q = {q.w, -q.x, -q.y, -q.z};
	Matrix<3,3> rot = { q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z, 2*(q.x*q.y + q.w*q.z), 2*(q.x*q.z - q.w*q.y),
										2*(q.x*q.y - q.w*q.z), q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z, 2*(q.y*q.z + q.w*q.x),
										2*(q.x*q.z + q.w*q.y), 2*(q.y*q.z - q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z };
	return rot*meas;
}