#include "KalmanFilter.h"

const float sigGy = 2.2e-8f; //approx PSD of gyroscope noise (from matlab)
const float sigMg = 4.35091e-5f;  //sigma squared of mag noise

KalmanFilter::KalmanFilter(Sensors& sensor, float latitude) : sens(sensor) {
  this->dipAngle = atanf(2.f*tanf(latitude*DEG_TO_RAD))-(-0.3759f); //66.5*DEG_TO_RAD;
  this->I10 = Matrix<float,10,10>::Identity();
};

void KalmanFilter::init() { //set initial values for filter
	R << sigMg,  0.f ,  0.f,
        0.f , sigMg,  0.f,
        0.f ,  0.f , sigMg;
  P = I10;

  aBase = {0.f, 0.f, -1.f};
  aInit = sens.aBase.normalized();

  float d = aInit.dot(aBase);   //construct quaternion from vertical and initial accelerometer vectors
  Vector3f c = aInit.cross(aBase);
  x << d+sqrtf(d*d + c.dot(c)), c, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f;
  Vector4f q = {x(0), x(1), x(2), x(3)}; q.normalize();
  Vector<float,10> xNew = {q(0), q(1), q(2), q(3), x(4), x(5), x(6), x(7), x(8), x(9)};
  x = xNew;

  //mBase = {-sinf(dipAngle), 0.f, cosf(dipAngle)}; //construct magnetometer base vector in ENU frame
  //mBase = rotate(x, mBase).normalized();
  mBase = {-0.07956, 0.42648, 0.90099};
  //mBase = {cosf(dipAngle),0.f,sinf(dipAngle)};
  sens.prev_filtered_mX = sens.mX; sens.prev_filtered_mY = sens.mY; sens.prev_filtered_mZ = sens.mZ; sens.prev_mX = sens.mX; sens.prev_mY = sens.mY; sens.prev_mZ = sens.mZ;
  sens.magFilter = true;
}

void KalmanFilter::predict(float dT) {
	Vector3f w = {sens.gX,sens.gY,sens.gZ};
  Vector3f a = {sens.aX,sens.aY,sens.aZ};
  a = invRotate(x, a);
  a(2) += 9.80665;
  Serial.print(a(0),5); Serial.print(", "); Serial.print(a(1),5); Serial.print(", "); Serial.println(a(2),5);
  a = rotate(x, a);
  float X = w(0)*dT/2.f; float Y = w(1)*dT/2.f; float Z = w(2)*dT/2.f;
  quaternion qPrev = {x(0), x(1), x(2), x(3)};

  x_pred = {qPrev.w - X*qPrev.x - Y*qPrev.y - Z*qPrev.z,
            qPrev.x + X*qPrev.w + Z*qPrev.y - Y*qPrev.z,
            qPrev.y + Y*qPrev.w - Z*qPrev.x + X*qPrev.z,
            qPrev.z + Z*qPrev.w + Y*qPrev.x - X*qPrev.y,
            x(4)+a(0)*dT, 
            x(5)+a(1)*dT, 
            x(6)+a(2)*dT, 
            x(7)+x(4)*dT+0.5f*a(0)*dT*dT,
            x(8)+x(5)*dT+0.5f*a(1)*dT*dT, 
            x(9)+x(6)*dT+0.5f*a(2)*dT*dT}; //predicted state estimate

  Vector4f q = {x_pred(0), x_pred(1), x_pred(2), x_pred(3)}; q.normalize();
  Vector<float,10> xNew = {q(0), q(1), q(2), q(3), x_pred(4), x_pred(5), x_pred(6), x_pred(7), x_pred(8), x_pred(9)};
  x_pred = xNew;
  
  F << 1.f, -X , -Y , -Z , 0.f, 0.f, 0.f, 0.f, 0.f, 0.f,
        X , 1.f,  Z , -Y , 0.f, 0.f, 0.f, 0.f, 0.f, 0.f,
        Y , -Z , 1.f,  X , 0.f, 0.f, 0.f, 0.f, 0.f, 0.f,
        Z ,  Y , -X , 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f,
       0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f,
       0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f,
       0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f,
       0.f, 0.f, 0.f, 0.f, dT , 0.f, 0.f, 1.f, 0.f, 0.f,
       0.f, 0.f, 0.f, 0.f, 0.f, dT , 0.f, 0.f, 1.f, 0.f,
       0.f, 0.f, 0.f, 0.f, 0.f, 0.f, dT , 0.f, 0.f, 1.f; //state transition matrix
  
  W <<  -qPrev.x, -qPrev.y, -qPrev.z, 0.f, 0.f, 0.f,
         qPrev.w, -qPrev.z,  qPrev.y, 0.f, 0.f, 0.f,
         qPrev.z,  qPrev.w, -qPrev.x, 0.f, 0.f, 0.f,
        -qPrev.y,  qPrev.x,  qPrev.w, 0.f, 0.f, 0.f,
            0.f ,     0.f ,     0.f , dT , 0.f, 0.f,
            0.f ,     0.f ,     0.f , 0.f, dT , 0.f,
            0.f ,     0.f ,     0.f , 0.f, 0.f, dT,
            0.f ,     0.f ,     0.f , dT*dT/2, 0.f, 0.f,
            0.f ,     0.f ,     0.f , 0.f, dT*dT/2, 0.f,
            0.f ,     0.f ,     0.f , 0.f, 0.f, dT*dT/2; //process noise jacobian
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
  H << rx*q.w+ry*q.z-rz*q.y, rx*q.y+ry*q.y+rz*q.z,-rx*q.y+ry*q.x-rz*q.w,-rx*q.z+ry*q.w+rz*q.x, 0,0,0,0,0,0,
      -rx*q.z+ry*q.w+rz*q.x, rx*q.y-ry*q.x+rz*q.w, rx*q.x+ry*q.y+rz*q.z,-rx*q.w-ry*q.z+rz*q.y, 0,0,0,0,0,0,
       rx*q.y-ry*q.x+rz*q.w, rx*q.z-ry*q.w-rz*q.x, rx*q.w+ry*q.z-rz*q.y, rx*q.x+ry*q.y+rz*q.z, 0,0,0,0,0,0; //measurement model jacobian
  //H *= 2.f;
  v = z - h; //innovation
  S = H*P_pred*H.transpose() + R; //innovation covariance
  K = P_pred*H.transpose()*Inverse(S); //kalman gain

  // Serial.print(m(0),5); Serial.print(", "); Serial.print(m(1),5); Serial.print(", "); Serial.println(m(2),5);
  
  x = x_pred + K*v; //updated state estimate
  Vector4f Q = {x(0), x(1), x(2), x(3)}; Q.normalize();
  Vector<float,10> xNew = {Q(0), Q(1), Q(2), Q(3), x(4), x(5), x(6), x(7), x(8), x(9)};
  x = xNew;
  
  P = (I10 - K*H)*P_pred; //updated state covariance
}

void KalmanFilter::filter(float dT) {
  predict(dT);
  correct();
}

Vector3f KalmanFilter::rotate(Vector<float,10> A, Vector3f meas) {
  quaternion q = {A(0), A(1), A(2), A(3)};
	Matrix3f rot {{q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z, 2*(q.x*q.y + q.w*q.z), 2*(q.x*q.z - q.w*q.y)},
								{2*(q.x*q.y - q.w*q.z), q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z, 2*(q.y*q.z + q.w*q.x)},
								{2*(q.x*q.z + q.w*q.y), 2*(q.y*q.z - q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z}};
	return rot*meas;
}

Vector3f KalmanFilter::invRotate(Vector<float,10> A, Vector3f meas) {
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