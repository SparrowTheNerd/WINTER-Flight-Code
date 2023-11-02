#pragma once

#include <SparkFunLSM9DS1.h>
#include <Adafruit_ADXL375.h>
#include <MS5xxx.h>
#include <eigen-3.4.0/Eigen/Dense>
using Eigen::Matrix; using Eigen::Vector;

class Sensors{
	public:
		Sensors( Vector<float,3> magHard, Matrix<float,3,3> magSoft, float xOfst, float yOfst, float zOfst);
		void init();
		void getData();
		void baroData();
		float altCalc();
		float aX, aY, aZ, gX, gY, gZ, mX, mY, mZ, alt, xOfst, yOfst, zOfst;
		float altInit = 0;
		uint32_t timeBaro;
		bool baroAvail;
		bool magAvail;
		Vector<float,3> magCal_hard;
		Matrix<float,3,3> magCal_soft;

	private:
		Adafruit_ADXL375 IMU_HighG = Adafruit_ADXL375(1,&Wire);
		MS5xxx barometer = MS5xxx(&Wire);
		LSM9DS1 IMU;
		int baroStep;
		unsigned long d1, d2;
		float prs;

};