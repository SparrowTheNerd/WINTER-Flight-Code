#include "KalmanFilter.h"

const float sigGy = 2.2e-8f; //approx PSD of gyroscope noise (from matlab)
const float sigMg = 4.35091e-5f;  //sigma squared of mag noise

KalmanFilter::KalmanFilter(Sensors& sensor, float latitude) : sens(sensor) {
  this->dipAngle = atanf(2.f*tanf(latitude*DEG_TO_RAD))-(-0.3759f); //66.5*DEG_TO_RAD;
  this->I4 = Matrix4f::Identity();
};

void KalmanFilter::init() { //set initial values for filter
	R << sigMg,  0.f ,  0.f,
        0.f , sigMg,  0.f,
        0.f ,  0.f , sigMg;
  P = I4;

  aBase = {1.f, 0.f, 0.f};
  aInit = sens.aBase.normalized();

  float d = aInit.dot(aBase);   //construct quaternion from vertical and initial accelerometer vectors
  Vector3f c = aInit.cross(aBase);
  x << d+sqrtf(d*d + c.dot(c)), c;
  x.normalize();

  mBase = {-sinf(dipAngle), 0.f, cosf(dipAngle)}; //construct magnetometer base vector in ENU frame
  mBase = rotate(x, mBase).normalized();
  sens.prev_filtered_mX = sens.mX; sens.prev_filtered_mY = sens.mY; sens.prev_filtered_mZ = sens.mZ; sens.prev_mX = sens.mX; sens.prev_mY = sens.mY; sens.prev_mZ = sens.mZ;
  sens.magFilter = true;
}

void KalmanFilter::predict(float dT) {
	Vector3f w = {sens.gX,sens.gY,sens.gZ};
  float X = w(0)*dT/2.f; float Y = w(1)*dT/2.f; float Z = w(2)*dT/2.f;

  F << 1.f, -X, -Y, -Z,
       X, 1.f, Z, -Y,
       Y, -Z, 1.f, X,
       Z, Y, -X, 1.f;  //state transition matrix

  x_pred = F*x; //predicted state estimate
  x_pred.normalize();
  quaternion qPrev = {x(0), x(1), x(2), x(3)};
  W << -qPrev.x, -qPrev.y, -qPrev.z,
        qPrev.w, -qPrev.z, qPrev.y,
        qPrev.z, qPrev.w, -qPrev.x,
        -qPrev.y, qPrev.x, qPrev.w; //process noise jacobian
  W *= dT/2.f;
  P_pred = F*P*(F.transpose()) + W*(W.transpose())*sigGy; //predicted state covariance. Pretending the process noise doesn't exist for now
}

void KalmanFilter::correct() {  
  Vector3f m_hat = rotate(x_pred, mBase); //predicted magnetometer measurement
  h = m_hat.normalized(); //measurement model
  m = {sens.mX, sens.mY, sens.mZ}; m.normalize(); //magnetometer measurement
  z = m; //measurement
  quaternion q = {x_pred(0), x_pred(1), x_pred(2), x_pred(3)};
  float rx = mBase(0); float ry = mBase(1); float rz = mBase(2);
  H << rx*q.w+ry*q.z-rz*q.y, rx*q.x+ry*q.y+rz*q.z, -rx*q.y+ry*q.x-rz*q.w, -rx*q.z+ry*q.w+rz*q.x,
      -rx*q.z+ry*q.w+rz*q.x, rx*q.y-ry*q.x+rz*q.w, rx*q.x+ry*q.y+rz*q.z, -rx*q.w-ry*q.z+rz*q.y,
       rx*q.y-ry*q.x+rz*q.w, rx*q.z-ry*q.w-rz*q.x, rx*q.w+ry*q.z-rz*q.y, rx*q.x+ry*q.y+rz*q.z; //measurement model jacobian
  H *= 2.f;
  v = z - h; //innovation
  S = H*P_pred*H.transpose() + R; //innovation covariance
  K = P_pred*H.transpose()*Inverse(S); //kalman gain
  
  x = x_pred + K*v; //updated state estimate
  x.normalize();
  
  P = (I4 - K*H)*P_pred; //updated state covariance
}

void KalmanFilter::filter(float dT) {
  predict(dT);
  correct();
}

Vector3f KalmanFilter::rotate(Vector4f A, Vector3f meas) {
  quaternion q = {A(0), A(1), A(2), A(3)};
	Matrix3f rot {{q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z, 2*(q.x*q.y + q.w*q.z), 2*(q.x*q.z - q.w*q.y)},
								{2*(q.x*q.y - q.w*q.z), q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z, 2*(q.y*q.z + q.w*q.x)},
								{2*(q.x*q.z + q.w*q.y), 2*(q.y*q.z - q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z}};
	return rot*meas;
}

Vector3f KalmanFilter::invRotate(Vector4f A, Vector3f meas) {
  quaternion q = {A(0), A(1), A(2), A(3)};
  Matrix3f rot {{q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z, 2*(q.x*q.y + q.w*q.z), 2*(q.x*q.z - q.w*q.y)},
								{2*(q.x*q.y - q.w*q.z), q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z, 2*(q.y*q.z + q.w*q.x)},
								{2*(q.x*q.z + q.w*q.y), 2*(q.y*q.z - q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z}};
  rot.transposeInPlace();
  return rot*meas;

}

int KalmanFilter::sign(float x) {
    return (x > 0) - (x < 0);
}