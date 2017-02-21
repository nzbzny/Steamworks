/*
 * Accumuator.h
 *
 *  Created on: Feb 7, 2017
 *      Author: FRC3941
 */
#include "WPILib.h"
#include "Constants.h"
#include "Aimer.h"
#include "PIDLoop.h"
#include <vector>

#ifndef SRC_ACCUMUATOR_H_
#define SRC_ACCUMUATOR_H_

struct DoubleDouble
{
	DoubleDouble(float xt, float yt, float zt): x(xt), y(yt), angle(zt) {}
	float x, y, angle;
};

class Accumulator
{

private:
	float predict(float power);
	void update(float encoder, float power, float powerX, float angle);
	void updateUS(float ultrasonic, int id);
	void updateX(float currentAngleError);
	inline DoubleDouble getCurrentPosition() {DoubleDouble current = history.back(); current.y += error; current.x += errorX; return current;}
	float GetXHistoryOffset(float targetAngle);
	std::vector<DoubleDouble> history;
	std::vector<int> lastUpdated;
	float error, errorX, prevPredicted, lastLeftProxValue, lastRightProxValue, lastEncoderDistance, lastCameraAngle;
	Ultrasonic *leftProx, *rightProx;
	Aimer *aimer;
	Encoder *encoder;
	PIDLoop *pid;

public:
	Accumulator() : history(), lastUpdated(), leftProx(NULL), rightProx(NULL), aimer(NULL), encoder(NULL), pid(NULL), error(0), prevPredicted(0), lastLeftProxValue(0), lastRightProxValue(0), lastEncoderDistance(0), lastCameraAngle(0), errorX(0) {history.push_back(DoubleDouble(0,0,0)); lastUpdated.resize(0);}
	Accumulator(float x, float y, float z, int usNum, Ultrasonic& left, Ultrasonic& right, Aimer& aim, Encoder& encod, PIDLoop& pd):
		history(), lastUpdated(), leftProx(&left),
		rightProx(&right), aimer(&aim), encoder(&encod),
		pid(&pd), error(0), prevPredicted(0),
		lastLeftProxValue(left.GetRangeInches()),
		lastRightProxValue(right.GetRangeInches()),
		lastEncoderDistance(0), lastCameraAngle(), errorX(0)
	{history.push_back(DoubleDouble(x,y,z)); for(int i = 0; i <= usNum; ++i) lastUpdated.push_back(0);}
	DoubleDouble drive(bool hasBeenGoing, bool encoderPos, bool powerPos, bool goingPos, float target, float targetAngle, float targetError, float robotAngle);
	void reset(float x, float y, float z);

};


#endif /* SRC_ACCUMUATOR_H_ */
