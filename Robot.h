#include "WPILib.h"
#include "Constants.h"
#include "PIDLoop.h"
#include "Aimer.h"
#include "AHRS.h"
#include "GearSubsystem.h"
#include "ShooterSubsystem.h"
#include "IR.h"
#include "Filters.h"
#include "IntakeSubsystem.h"
#include "ClimberSubsystem.h"
#include "Brakes.h"
#include "CANTalon.h"
#include "Accumulator.h"
#include <math.h>
#include <thread>
#include <fstream>

#define PI 3.14159265

#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_

class Robot : public SampleRobot {

	CANTalon frontLeftMotor;
	CANTalon rearLeftMotor;
	CANTalon frontRightMotor;
	CANTalon rearRightMotor; //TODO: reinitialize for new drive train
	frc::RobotDrive robotDrive;
	frc::Joystick driveStick;
	frc::Joystick operatorStick;
	AHRS gyro;
	PIDLoop pid;
	Aimer aimer;
	Ultrasonic leftProx;
	Ultrasonic rightProx;
	IR leftIR;
	IR rightIR;
	GearSubsystem gear;
	ShooterSubsystem shooter;
	Compressor compressor;
	Filters filter;
	IntakeSubsystem intake;
	ClimberSubsystem climber;
	Brakes brakes;
	PowerDistributionPanel pdp;
	Encoder* encoder;
	Encoder yEncoder;
	Encoder xEncoder;
	cs::UsbCamera camera0;
	cs::UsbCamera camera1;
	I2C arduino;

public:
	Robot();
	void RobotInit();
	void OperatorControl();
	void Autonomous();
};

#endif /* SRC_ROBOT_H_ */
