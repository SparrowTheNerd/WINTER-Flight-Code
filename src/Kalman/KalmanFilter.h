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
		
		

	private:
		

};