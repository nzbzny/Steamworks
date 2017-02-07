#include <Filters.h>

Filters::Filters():
history()
{
	refreshRate = .01;
	lastLeftUltrasonic = 0.0;
	lastRightUltrasonic = 0.0;
	predictedValue = 0.0;
}

void Filters::update(float x, float y, float angle)
{
	history.push_back(DoubleDouble(x,y,angle));
}

void Filters::initializeLastUltrasonics(float left, float right) {
	lastLeftUltrasonic = left;
	lastRightUltrasonic = right;
}

void Filters::initializePredictedValue(float left, float right) {
	predictedValue = ultrasonicFilter(left, right);
}

float Filters::ultrasonicFilter(float left, float right) {
	float leftOutput = 0;
	float rightOutput = 0;
	float gain = .2;
	if (left > 0 && left < 120 && right > 0 && right < 120) { //if both distances are within range to go to a gear
		leftOutput = gain * left + (1 - gain) * lastLeftUltrasonic; //larry math
		rightOutput = gain * right + (1 - gain) * lastRightUltrasonic;

		lastLeftUltrasonic = left;
		lastRightUltrasonic = right;

		return (leftOutput + rightOutput) / 2; //average
		SmartDashboard::PutString("PIDY Avg. Dist. Loop", "ultrasonicFilter");
	} else if ((left < 0 || left > 120) && (right > 0 && right < 120)) { //if left sensor is outside of range to move to gear
		return right;
		SmartDashboard::PutString("PIDY Avg. Dist. Loop", "right");
	} else if ((right < 0 || right > 120) && (left > 0 && left < 120)) { //if right sensor is outside of range to move to gear
		return left;
		SmartDashboard::PutString("PIDY Avg. Dist. Loop", "left");
	} else {
		SmartDashboard::PutString("PIDY Status", "Ultrasonic Error");
		return -1;
	}
}

float Filters::kalmanFilter(float left, float right, float power) {
	float ultrasonicFilterVar;
	if (fabs(left - lastLeftUltrasonic) < 0.01 && fabs(right - lastRightUltrasonic) < 0.01) {
		predictedValue += 90 * power * refreshRate; //90 inches * power * time (-power means going forward) TODO: flip sign if necessary
		return predictedValue;
	}
	if (fabs(left - lastLeftUltrasonic) > 0.01) {
		ultrasonicFilterVar = ultrasonicFilter(left, right);
		predictedValue = predictedValue + (ultrasonicFilterVar - history[lastLeftUpdatedElement].y);
		lastLeftUpdatedElement = history.size() - 1;
	}
	if (fabs(left - lastLeftUltrasonic) < 0.01) {
		ultrasonicFilterVar = ultrasonicFilter(left, right);
		predictedValue = predictedValue + (ultrasonicFilterVar - history[lastRightUpdatedElement].y);
		lastRightUpdatedElement = history.size() - 1;
	}
	return predictedValue;
}

//predictedValue = predictedValue + (ultrasonicFilterVar - lastUpdatedPredictedValue); //ultrasonic filter - lastValue is the difference in the actual which we use to update our predicted
//predictedValue += 90 * power * refreshRate; //90 inches * power * time (-power means going forward) TODO: flip sign if necessary
