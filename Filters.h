#include "WPILib.h"
#include <vector>

#ifndef SRC_FILTERS_H
#define SRC_FILTERS_H

struct DoubleDouble
{
	DoubleDouble(float xt, float yt, float zt): x(xt), y(yt), angle(zt) {}
	float x, y, angle;
};

class Filters {
private:
	//Kalman filter
	float refreshRate;
	float predictedValue;

	float lastLeftUltrasonic;
	float lastRightUltrasonic;
	int lastLeftUpdatedElement;
	int lastRightUpdatedElement;

	std::vector<DoubleDouble> history;

public:
	Filters();
	void initializeLastUltrasonics(float left, float right);
	void initializePredictedValue(float left, float right);
	float ultrasonicFilter(float left, float right);
	float kalmanFilter(float left, float right, float power);
	void update(float x, float y, float angle);
};

#endif
