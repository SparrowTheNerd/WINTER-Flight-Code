#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(Sensors &sensor) : sens(sensor) {};

void KalmanFilter::init(float stddev_mg, float stddev_ps) {
  sens.getData();

  while(!sens.baroAvail) {
    sens.baroData();
  }
  sens.altInit = sens.altCalc();

  Rmg = { stddev_mg,     0    ,     0     ,
              0    , stddev_mg,     0     ,
              0    ,     0    , stddev_mg };
  Rbm = { stddev_ps };

  Hbm = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};		//measurement matrices
  Hmg = { 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 1};

  mgBase = {sens.mX,sens.mY,sens.mZ};
  Matrix<3> gravVect = {sens.aX, sens.aY, sens.aZ};
  gravVect /= sqrt(sens.aX*sens.aX+sens.aY*sens.aY+sens.aZ*sens.aZ);      //normalize gravity vector
  float psi = M_PI_2-(acos(gravVect(1)));
  float phi = M_PI_2-(acos(gravVect(2)));
  float theta = 0.;
  quaternion q;
  q.r = sin(phi/2.)*cos(theta/2.)*cos(psi/2.)-cos(phi/2.)*sin(theta/2.)*sin(psi/2.);    //convert initial orientation into state quaternion
  q.i = cos(phi/2.)*cos(theta/2.)*cos(psi/2.)+sin(phi/2.)*sin(theta/2.)*sin(psi/2.);
  q.j = cos(phi/2.)*sin(theta/2.)*cos(psi/2.)+sin(phi/2.)*cos(theta/2.)*sin(psi/2.);
  q.k = cos(phi/2.)*cos(theta/2.)*sin(psi/2.)-sin(phi/2.)*sin(theta/2.)*cos(psi/2.);

  x = {0., 0., 0., 0., 0., 0., q.r, q.i, q.j, q.k};   //initial state estimate; position and velocity are zero, and orientation calculated above
  P.Fill(0.);   //initial covariance error is zero, initial estimate is as close to true state as is reasonable
  
    
}

void KalmanFilter::gain() {
  /* magnetometer section */
  Matrix<10,4> Hmg_T = ~Hmg;
  Kmg = P_prior*Hmg_T*Inverse(Hmg*P_prior*Hmg_T+Rmg);
  /* barometer section */
  Matrix<10> Hbm_T = ~Hbm;
  Kbm = P_prior*Hbm_T*Inverse(Hbm*P_prior*Hbm_T+Rbm);
}
void KalmanFilter::update(bool magAvail, bool baroAvail) {    //state update using mag and baro & update covariance
  x = x_prior;
  if(magAvail) {
    float magMag = sqrtf(sens.mX*sens.mX+sens.mY*sens.mY+sens.mZ*sens.mZ);    //magnetometer magnitude
    zMg(0) = (acos(sens.mX/magMag)-acos(mgBase(0)));
    zMg(1) = (acos(sens.mY/magMag)-acos(mgBase(1)));
    zMg(2) = (acos(sens.mZ/magMag)-acos(mgBase(2)));
    x += Kmg*(zMg - Hmg*x_prior);
  }
  if(baroAvail) {
    x += Kbm*(zBm - Hbm*x_prior);
  }

  Matrix<10,10> I; I.Fill(0.);
  for(int i=0; i<10; i++) {   //identity matrix
    I(i,i) = 1.;
  }
  P = (I-Kbm*Hbm)*P_prior*~(I-Kbm*Hbm) + Kbm*Rbm*~Kbm;
  P+= (I-Kmg*Hmg)*P_prior*~(I-Kmg*Hmg) + Kmg*Rmg*~Kmg;
}
void KalmanFilter::extrapolate(float dT) {   //extrapolation / prediction function
  float tht = (uGy(0));
  float phi = (uGy(1));
  float psi = (uGy(2));
  G = { dT*dT/2,    0   ,    0   ,  0 ,
           0   , dT*dT/2,    0   ,  0 ,
           0   ,    0   , dT*dT/2,  0 ,
           dT  ,    0   ,    0   ,  0 ,
           0   ,    dT  ,    0   ,  0 ,
           0   ,    0   ,    dT  ,  0 ,
           0   ,    0   ,    0   ,  0 ,
           0   ,    0   ,    0   ,  0 ,
           0   ,    0   ,    0   ,  0 ,
           0   ,    0   ,    0   ,  0 };
  
  F = { 1, 0, 0, dT,  0,  0, 0, 0, 0, 0,
        0, 1, 0,  0, dT,  0, 0, 0, 0, 0,
        0, 0, 1,  0,  0, dT, 0, 0, 0, 0,
        0, 0, 0,  1,  0,  0, 0, 0, 0, 0,
        0, 0, 0,  0,  1,  0, 0, 0, 0, 0,
        0, 0, 0,  0,  0,  1, 0, 0, 0, 0,
        0, 0, 0,  0,  0,  0, 1, 0, 0, 0,
        0, 0, 0,  0,  0,  0, 0, 1, 0, 0,
        0, 0, 0,  0,  0,  0, 0, 0, 1, 0,
        0, 0, 0,  0,  0,  0, 0, 0, 0, 1};
  
  //quaternion angular rates: https://mariogc.com/post/angular-velocity-quaternions/ 
  Matrix<4> q0 = {x(6),x(7),x(8),x(9)};
  Matrix<4,4> omega = {  0 , psi , -phi, tht,
                            -psi,  0  ,  tht, phi,
                             phi, -tht,   0 , psi,
                            -tht, -phi, -psi,  0 };
  Matrix<4> qd = omega * q0 * 0.5f * dT;

  x_prior = (F*x + G*uXl);
  x_prior(6) += qd(0); x_prior(7) += qd(1); x_prior(8) += qd(2); x_prior(9) += qd(3);
  // Serial << x_prior;
  // Serial.print('\n');
  P_prior = F*P*~F; 
}

void KalmanFilter::normalize() {
  float normFact = sqrt(x(6)*x(6) + x(7)*x(7) + x(8)*x(8) + x(9)*x(9));
  if (normCounter >= 5) {
    x(6) /= normFact;
    x(7) /= normFact;
    x(8) /= normFact;
    x(9) /= normFact;
		normCounter = 0;
  }
	normCounter++;
}

void KalmanFilter::filter(float dT) {
	normalize();
	extrapolate(dT);
	gain();
	update(sens.magAvail, sens.baroAvail);
}