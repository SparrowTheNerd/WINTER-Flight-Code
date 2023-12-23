#pragma once

#include <SparkFunLSM9DS1.h>
#include <Adafruit_ADXL375.h>
#include <MS5xxx.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

class Sensors{
	public:
		Sensors( Matrix<3> magHard, Matrix<3,3> magSoft);
		void init();
		void getData();
		void baroData();
		float altCalc();
		float aX, aY, aZ, gX, gY, gZ, mX, mY, mZ, alt, xOfst, yOfst, zOfst;
		float altInit = 0;
		uint32_t timeBaro;
		bool baroAvail;
		bool magAvail;
		Matrix<3> magCal_hard;
		Matrix<3,3> magCal_soft;
		LSM9DS1 IMU;
		float dt;
		float prev_mX, prev_mY, prev_mZ, prev_filtered_mX, prev_filtered_mY, prev_filtered_mZ;
		bool magFilter = false;

	private:
		Adafruit_ADXL375 IMU_HighG = Adafruit_ADXL375(1,&Wire);
		MS5xxx barometer = MS5xxx(&Wire);
		
		int baroStep;
		unsigned long d1, d2;
		float prs;

};