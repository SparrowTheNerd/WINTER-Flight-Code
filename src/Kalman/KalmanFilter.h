#pragma once

#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include "Sensors/Sensors.h"
using namespace BLA;

class KalmanFilter {
	public:
		struct quaternion {
			float w;          //real value
			float x, y, z;    //imaginary values
		};
		KalmanFilter(Sensors &sens);
		void init();
		void filter(float dT);
		Matrix<4> x;

	private:
		Sensors &sens;
		Matrix<4,4> F;
		Matrix<3> w;
		Matrix<3> rotate(quaternion q, Matrix<3> w);

};