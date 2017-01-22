#include "WPILib.h"
#include "Constants.h"
#include "PIDLoop.h"
#include "Aimer.h"
#include "AHRS.h"
#include "GearSubsystem.h"
#include "ShooterSubsystem.h"
#include "CANTalon.h"
#include <math.h>
#include <thread>
#include <fstream>

#define PI 3.14159265

#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_

class Robot : public SampleRobot {

	CANTalon frontLeftMotor;
	CANTalon frontRightMotor;
	CANTalon rearLeftMotor;
	CANTalon rearRightMotor;
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
	GearSubsystem gear;
	ShooterSubsystem shooter;
	Compressor compressor;

public:
	Robot();
	void RobotInit();
	void OperatorControl();
	void Autonomous();
};

#endif /* SRC_ROBOT_H_ */
