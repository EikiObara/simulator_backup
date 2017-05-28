// 2017/04/28
// created by eiki obara

#include <stdio.h>
//#include "include\trapezoidal_interpolation.h"
#include "include\kine_debag.h"
#include "include\kine_config.h"

double TrapeInterpolate(const double move_length, const double time_length, const double currentTime) {
	//DebagComment("trape interpolate");

	double buf = 0;

	if (currentTime < kine::ACCEL_TIME) {
		buf = kine::TIME_SPAN * (currentTime / kine::ACCEL_TIME) * (move_length /(kine::TIME_LENGTH - kine::ACCEL_TIME));
		return buf;
	}
	else if (currentTime >= kine::ACCEL_TIME && currentTime <= (time_length - kine::ACCEL_TIME)) {
		buf = kine::TIME_SPAN * ((move_length) / (time_length - kine::ACCEL_TIME));
		return buf;
	}
	else if (currentTime >(time_length - kine::ACCEL_TIME) && currentTime <= time_length) {
		buf = kine::TIME_SPAN * (move_length * (time_length - currentTime)) / ((time_length - kine::ACCEL_TIME) * kine::ACCEL_TIME);
		return buf;
	}
	else if(currentTime > time_length){
		return 0.0;
	}
}