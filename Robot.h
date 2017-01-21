#include "WPILib.h"
#include "Constants.h"
#include "PIDLoop.h"
#include "Aimer.h"
#include "AHRS.h"
#include <math.h>
#include <thread>
#include <fstream>

#define PI 3.14159265

#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_

class Robot : public SampleRobot {

	frc::RobotDrive robotDrive;
	frc::Joystick driveStick;
	frc::Joystick operatorStick;
	AHRS gyro;
	PIDLoop pid;
	Aimer aimer;
	Ultrasonic leftProx;
	Ultrasonic rightProx;
	DigitalInput leftIR;
	DigitalInput rightIR;

public:
	Robot();
	void RobotInit();
	void OperatorControl();
	void Autonomous();
};

#endif /* SRC_ROBOT_H_ */
