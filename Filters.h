#include "WPILib.h"

#ifndef SRC_FILTERS_H
#define SRC_FILTERS_H

class Filters {

	float refreshRate;
	float lastUltrasonicValue;
	float predictedValue;

	//ultrasonic filter
	float lastLeftUltrasonic;
	float lastRightUltrasonic;

public:
	Filters();
	void initializeLastUltrasonics(float left, float right);
	void initializePredictedValue(float left, float right);
	float ultrasonicFilter(float left, float right);
	float kalmanFilter(float left, float right, float power);
};

#endif
