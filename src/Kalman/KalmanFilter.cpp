#include "KalmanFilter.h"
using namespace Eigen;

KalmanFilter::KalmanFilter(Sensors& sensor) : sens(sensor) {};

void KalmanFilter::init(float stddev_mg, float stddev_ps) {
  sens.getData();

  while(!sens.baroAvail) {
    sens.baroData();
  }
  sens.altInit = sens.altCalc();

  Rmg = { stddev_mg,     0    ,     0     ,     0     ,
              0    , stddev_mg,     0     ,     0     ,
              0    ,     0    , stddev_mg ,     0     ,
              0    ,     0     ,    0     , stddev_mg};
  Rbm(0,0) = stddev_ps;

  Hbm = { 0, 0, 1.f, 0, 0, 0, 0, 0, 0, 0};		//measurement matrices
  Hmg = { 0, 0, 0, 0, 0, 0, 1.f, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 1.f, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 1.f, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 1.f};

  mgBase = {sens.mX,sens.mY,sens.mZ};
  Vector<float, 3> gravVect = {sens.aX, sens.aY, sens.aZ};
	Vector<float, 3> gravBase = {1.f, 0.f, 0.f};																		//real gravity is straight on global x
  gravVect /= sqrt(sens.aX*sens.aX+sens.aY*sens.aY+sens.aZ*sens.aZ);      //normalize gravity vector
  
	float halfAng = acosf(gravVect(0)*gravBase(0) + gravVect(1)*gravBase(1) + gravVect(2)*gravBase(2)) / 2.f;			//generate quaternion between 
	Vector<float, 3> axis = gravBase.cross(gravVect);
	float sA = sinf(halfAng);
	quaternion q = {cosf(halfAng),axis(0)*sA, axis(1)*sA, axis(2)*sA};

  x = {0., 0., 0., 0., 0., 0., q.w, q.x, q.y, q.z};   //initial state estimate; position and velocity are zero, and orientation calculated above
	normalize();
  P.fill(0.);   //initial covariance error is zero, initial estimate is as close to true state as is reasonable
}

void KalmanFilter::gain() {
  /* magnetometer section */
  Matrix<float,10,4> Hmg_T = Hmg.transpose();
  Matrix<float, 4, 4> temp = (Hmg*P_prior*Hmg_T+Rmg);
  Kmg = (P_prior*Hmg_T*temp.inverse());

  /* barometer section */
  Vector<float, 10> Hbm_T = Hbm.transpose();
  Matrix<float, 1, 1> temp2 = (Hbm*P_prior*Hbm_T+Rbm);
  Kbm = P_prior*Hbm_T*temp2.inverse();
}
void KalmanFilter::update(bool magAvail, bool baroAvail) {    //state update using mag and baro & update covariance
  x = x_prior;
  if(magAvail) {
    float magMag = sqrtf(sens.mX*sens.mX+sens.mY*sens.mY+sens.mZ*sens.mZ);    //magnetometer magnitude
		Vector<float, 3> mag = {sens.mX/magMag, sens.mY/magMag, sens.mZ/magMag};	//normalize mag vector

		float halfAng = acosf(mag(0)*mgBase(0) + mag(1)*mgBase(1) + mag(2)*mgBase(2)) / 2.f;			//generate quaternion between 
		Vector<float, 3> axis = mgBase.cross(mag);
		float sA = sinf(halfAng);
		zMg = {cosf(halfAng),axis(0)*sA, axis(1)*sA, axis(2)*sA};

    x += Kmg*(zMg - Hmg*x_prior);
  }
  if(baroAvail) {
		zBm(0,0) = sens.altCalc();
    x += Kbm*(zBm - Hbm*x_prior);
  }

  Matrix<float, 10,10> I = Matrix<float, 10,10>::Identity();

  P = (I-Kbm*Hbm)*P_prior*((I-Kbm*Hbm).transpose()) + Kbm*Rbm*(Kbm.transpose());
  P+= (I-Kmg*Hmg)*P_prior*((I-Kmg*Hmg).transpose()) + Kmg*Rmg*(Kmg.transpose());
}
void KalmanFilter::extrapolate(float dT) {   //extrapolation / prediction function
  float X = uGy(0);
	float Y = uGy(1);
	float Z = uGy(2);
	quaternion q = {x(6),x(7),x(8),x(9)};
  G = { dT*dT/2.f,    0   ,    0   ,                 0 ,			//control matrix for acceleration and quat rates (acceleration -> velocity & pos)
           0   , dT*dT/2.f,    0   ,                 0 ,
           0   ,    0   , dT*dT/2.f,                 0 ,
           dT  ,    0   ,    0   ,                   0 ,
           0   ,    dT  ,    0   ,                   0 ,
           0   ,    0   ,    dT  ,                   0 ,
           0   ,    0   ,    0   ,  (dT/2.)*(q.w - q.x*X - q.y*Y - q.z*Z),	//quaternion rates
           0   ,    0   ,    0   ,  (dT/2.)*(q.w*X + q.x + q.y*Z - q.z*Y),
           0   ,    0   ,    0   ,  (dT/2.)*(q.w*Y - q.x*Z + q.y + q.z*X),
           0   ,    0   ,    0   ,  (dT/2.)*(q.w*Z + q.x*Y - q.y*X + q.z)};
  
  F = { 1.f, 0, 0, dT,  0,  0, 0, 0, 0, 0,		//state transition matrix (velocity -> position)
        0, 1.f, 0,  0, dT,  0, 0, 0, 0, 0,
        0, 0, 1.f,  0,  0, dT, 0, 0, 0, 0,
        0, 0, 0,  1.f,  0,  0, 0, 0, 0, 0,
        0, 0, 0,  0,  1.f,  0, 0, 0, 0, 0,
        0, 0, 0,  0,  0,  1.f, 0, 0, 0, 0,
        0, 0, 0,  0,  0,  0, 1.f, 0, 0, 0,
        0, 0, 0,  0,  0,  0, 0, 1.f, 0, 0,
        0, 0, 0,  0,  0,  0, 0, 0, 1.f, 0,
        0, 0, 0,  0,  0,  0, 0, 0, 0, 1.f};
	
	x_prior = F*x + G*uXl;
  P_prior = F*P*(F.transpose());
  //Serial.println((dT/2)*(q.w - q.x*X - q.y*Y - q.z*Z),6);
}

void KalmanFilter::normalize() {
  float normFact = sqrt(x(6)*x(6) + x(7)*x(7) + x(8)*x(8) + x(9)*x(9));
  //if (normCounter >= 5) {
    x(6) /= normFact;
    x(7) /= normFact;
    x(8) /= normFact;
    x(9) /= normFact;
		//normCounter = 0;
  //}
	//normCounter++;
}

Vector<float,3> KalmanFilter::measTransform(quaternion q, Vector<float, 3> meas) {
	//q.x = -q.x; q.y = -q.y; q.z = -q.z;
	Matrix<float,3,3> transform = { q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z, 2*(q.x*q.y - q.w*q.z), 2*(q.w*q.y + q.x*q.z),
													      	2*(q.x*q.y + q.w*q.z), q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z, 2*(q.y*q.z - q.w*q.x),
														      2*(q.x*q.z - q.w*q.y), 2*(q.w*q.x + q.y*q.z), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z};
  //Serial.print(q.w); Serial.print(","); Serial.print(q.x); Serial.print(","); Serial.print(q.y); Serial.print(","); Serial.println(q.z);
  return transform*meas;
}

void KalmanFilter::filter(float dT) {
	uXl << measTransform(quaternion {x(6),x(7),x(8),x(9)}, Vector<float,3> {sens.aX, sens.aY, sens.aZ}), 1;	//make 4 entry vector by concatenating transform with 1
	uGy = measTransform(quaternion {x(6),x(7),x(8),x(9)}, Vector<float,3> {sens.gX, sens.gY, sens.gZ});
	extrapolate(dT);
	gain();
	update(sens.magAvail, sens.baroAvail);
  normalize();
  //Serial.print(x(6),4); Serial.print(" , "); Serial.print(x(7),4); Serial.print(" , "); Serial.print(x(8),4); Serial.print(" , "); Serial.println(x(9),4);
}