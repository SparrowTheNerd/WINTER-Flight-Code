#pragma once

#include <SparkFunLSM9DS1.h>
#include <Adafruit_ADXL375.h>
#include <MS5xxx.h>
#include <ArduinoEigenDense.h>
using namespace Eigen;
//#include <BasicLinearAlgebra.h>
//using namespace BLA;

class Sensors{
	public:
		Sensors( Vector3f magHard, Matrix3f magSoft, Vector3f aBias);
		void init();
		void getData();
		void baroData();
		float altCalc();
		float aX, aY, aZ, gX, gY, gZ, mX, mY, mZ, alt, xOfst, yOfst, zOfst;
		Vector3f magBase;
		Vector3f aBase;
		Vector3f accelBias;
		float altInit = 0;
		uint32_t timeBaro;
		bool baroAvail;
		bool magAvail;
		Vector3f magCal_hard;
		Matrix3f magCal_soft;
		LSM9DS1 IMU;
		float dt;
		float prev_mX, prev_mY, prev_mZ, prev_filtered_mX, prev_filtered_mY, prev_filtered_mZ;
		bool magFilter = false;

	private:
		Adafruit_ADXL375 IMU_HighG = Adafruit_ADXL375(1,&Wire);
		MS5xxx barometer = MS5xxx(&Wire);
		Vector<float,3> mag;
		
		int baroStep;
		unsigned long d1, d2;
		float prs;

};