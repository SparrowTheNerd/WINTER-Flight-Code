#include "KalmanFilter.h"



KalmanFilter::KalmanFilter(Sensors& sensor) : sens(sensor) {};

Matrix<3> KalmanFilter::crossProd(Matrix<3> A, Matrix<3> B) {
	float c1 = A(1) * B(2) - B(1) * A(2);
	float c2 = A(2) * B(0) - B(2) * A(0);
	float c3 = A(0) * B(1) - B(0) * A(1);
	float magnitude = sqrtf(c1*c1+c2*c2+c3*c3);
	return Matrix<3> {c1/magnitude, c2/magnitude, c3/magnitude};	//normalize the cross product
}

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

  Hbm = { 1.f, 0, 0, 0, 0, 0, 0, 0, 0, 0};		//measurement matrices
  Hmg = { 0, 0, 0, 0, 0, 0, 1.f, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 1.f, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 1.f, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 1.f};

  Matrix<3> gravVect = {sens.aX, sens.aY, sens.aZ};
	Matrix<3> gravBase = {1.f, 0.f, 0.f};																		//real gravity is straight on global x
  gravVect /= sqrt(sens.aX*sens.aX+sens.aY*sens.aY+sens.aZ*sens.aZ);      //normalize gravity vector
  
	float halfAng = acosf(gravVect(0)*gravBase(0) + gravVect(1)*gravBase(1) + gravVect(2)*gravBase(2)) / 2.f;			//generate quaternion between 
	Matrix<3> axis = crossProd(gravBase,gravVect);
	float sA = sinf(halfAng);
	quaternion q = {cosf(halfAng),axis(0)*sA, axis(1)*sA, axis(2)*sA};

	float magMag = sqrtf(sens.mX*sens.mX+sens.mY*sens.mY+sens.mZ*sens.mZ);
  Matrix<3> magTemp = {sens.mX/magMag,sens.mY/magMag,sens.mZ/magMag};    //normalize the base mag vector
	mgBase = measTransform(q,magTemp);	//rotate mgBase so that the magnetometer takes the initial orientation into account

  x = {0., 0., 0., 0., 0., 0., q.w, q.x, q.y, q.z};   //initial state estimate; position and velocity are zero, and orientation calculated above
	normalize();
  P.Fill(0.001f);   //initial covariance error is zero, initial estimate is as close to true state as is reasonable
    
}

void KalmanFilter::gain() {
  /* magnetometer section */
  Matrix<10,4> Hmg_T = ~Hmg;
  Kmg = P_prior*Hmg_T*Inverse(Hmg*P_prior*Hmg_T+Rmg);
  /* barometer section */
  Matrix<10> Hbm_T = ~Hbm;
  Kbm = /*{0.1f, 0, 0, 0, 0, 0, 0, 0, 0, 0};*/ P_prior*Hbm_T*Inverse(Hbm*P_prior*Hbm_T+Rbm);
	//Serial << Kbm;
	//Serial.println();
}
void KalmanFilter::update() {    //state update using mag and baro & update covariance
  x = x_prior;
  // if(sens.magAvail) {
  //   float magMag = sqrtf(sens.mX*sens.mX+sens.mY*sens.mY+sens.mZ*sens.mZ);    //magnetometer magnitude
	// 	Matrix<3> mag = {sens.mX/magMag, sens.mY/magMag, sens.mZ/magMag};	//normalize mag vector

	// 	float halfAng = acosf(mag(0)*mgBase(0) + mag(1)*mgBase(1) + mag(2)*mgBase(2)) / 2.f;			//generate quaternion between 
	// 	Matrix<3> axis = crossProd(mgBase,mag);
	// 	float sA = sinf(halfAng);
	// 	zMg = {cosf(halfAng),axis(0)*sA, axis(1)*sA, axis(2)*sA};
  //   // Serial << mgBase; Serial.print("    ,    ");
  //   // Serial << mag; Serial.println();
	// 	// Serial << zMg; Serial.println();
	// 	//Serial << Kmg; Serial.println();

  //   x += Kmg*(zMg - Hmg*x_prior);
  // }
  // if(sens.baroAvail) {
	// 	zBm = {sens.altCalc()};
  //   x += Kbm*(zBm - Hbm*x_prior);
  // }

  // Matrix<10,10> I; I.Fill(0.f);
  // for(int i=0; i<10; i++) {   //identity matrix
  //   I(i,i) = 1.f;
  // }
  // P = (I-Kbm*Hbm)*P_prior*~(I-Kbm*Hbm) + Kbm*Rbm*~Kbm;
  //P+= (I-Kmg*Hmg)*P_prior*~(I-Kmg*Hmg) + Kmg*Rmg*~Kmg;
}
void KalmanFilter::extrapolate(float dT) {   //extrapolation / prediction function
	float X = uGy(0)*dT/2.f;
	float Y = uGy(1)*dT/2.f;
	float Z = uGy(2)*dT/2.f;
	G = { dT*dT/2,    0   ,    0   , 0, 0, 0, 0,			//control matrix for acceleration and quat rates (acceleration -> velocity & pos)
           0   , dT*dT/2,    0   , 0, 0, 0, 0,
           0   ,    0   , dT*dT/2, 0, 0, 0, 0,
           dT  ,    0   ,    0   , 0, 0, 0, 0,
           0   ,    dT  ,    0   , 0, 0, 0, 0,
           0   ,    0   ,    dT  , 0, 0, 0, 0,
           0   ,    0   ,    0   , 0,-X,-Y, -Z,
           0   ,    0   ,    0   , X, 0, Z, -Y,
           0   ,    0   ,    0   , Y,-Z, 0,  X,
           0   ,    0   ,    0   , Z, Y, -X, 0};
  
  F = { 1.f, 0, 0, dT,  0,  0, 0, 0, 0, 0,		//state transition matrix (velocity -> position)
        0, 1.f, 0,  0, dT,  0, 0, 0, 0, 0,
        0, 0, 1.f,  0,  0, dT, 0, 0, 0, 0,
        0, 0, 0,  1.f,  0,  0, 0, 0, 0, 0,
        0, 0, 0,  0,  1.f,  0, 0, 0, 0, 0,
        0, 0, 0,  0,  0,  1.f, 0, 0, 0, 0,
        0, 0, 0,  0,  0,  0, 1.f, 0, 0, 0,		//quaternion rates
        0, 0, 0,  0,  0,  0, 0, 1.f, 0, 0,
        0, 0, 0,  0,  0,  0, 0, 0, 1.f, 0,
        0, 0, 0,  0,  0,  0, 0, 0, 0, 1.f};
  
	
	x_prior = F*x + G*(uXl && Matrix<4> {x(6),x(7),x(8),x(9)});
  //x_prior(6) += qdot(0); x_prior(7) += qdot(1); x_prior(8) += qdot(2); x_prior(9) += qdot(3);
  P_prior = F*P*~F; 
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

Matrix<3> KalmanFilter::measTransform(quaternion q, Matrix<3> meas) {
	//q.x = -q.x; q.y = -q.y; q.z = -q.z;
	Matrix<3,3> transform = { q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z, 2*(q.x*q.y + q.w*q.z), 2*(q.x*q.z - q.w*q.y),
														2*(q.x*q.y - q.w*q.z), q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z, 2*(q.y*q.z + q.w*q.x),
														2*(q.x*q.z + q.w*q.y), 2*(q.y*q.z - q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z };
  //Serial.print(q.w); Serial.print(","); Serial.print(q.x); Serial.print(","); Serial.print(q.y); Serial.print(","); Serial.println(q.z);
  return transform*meas;
}

void KalmanFilter::filter(float dT) {
	uXl = measTransform(quaternion {x(6),x(7),x(8),x(9)}, Matrix<3> {sens.aX, sens.aY, sens.aZ});
	uGy = measTransform(quaternion {x(6),x(7),x(8),x(9)}, Matrix<3> {sens.gX, sens.gY, sens.gZ});
	uXl(0) -= 9.80665f;		//subtract gravity
	extrapolate(dT);
	gain();
	update();
  normalize();
  //Serial.print(x(6),4); Serial.print(" , "); Serial.print(x(7),4); Serial.print(" , "); Serial.print(x(8),4); Serial.print(" , "); Serial.println(x(9),4);
}