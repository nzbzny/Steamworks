#include "Robot.h"
#include "WPILib.h"

/*void moveToGearThreadFunction(bool *keepRunning, PIDLoop *pid) {
	while (*keepRunning == true) {
		if (pid->runPID() == 1) {
			*keepRunning = false;
		}
	}
}*/

Robot::Robot() :
		robotDrive(Constants::frontLeftDriveChannel, Constants::rearLeftDriveChannel, Constants::frontRightDriveChannel, Constants::rearRightDriveChannel),
		driveStick(Constants::driveStickChannel),
		operatorStick(Constants::operatorStickChannel),
		gyro(I2C::Port::kMXP, 200),
		pid(),
		aimer(),
		leftProx(1, 0),
		rightProx(3, 2),
		leftIR(4),
		rightIR(5)
{
	robotDrive.SetExpiration(0.1);
	robotDrive.SetInvertedMotor(RobotDrive::kFrontLeftMotor, false);
	robotDrive.SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
	robotDrive.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
	robotDrive.SetInvertedMotor(RobotDrive::kRearRightMotor, true);
}

void Robot::RobotInit() {}

/**
 * Runs the motors with Mecanum drive.
 */
void Robot::OperatorControl()
{
	pid.setAngle(SmartDashboard::GetNumber("angle_p", .015), SmartDashboard::GetNumber("angle_i", .001), SmartDashboard::GetNumber("angle_d", .001));
	robotDrive.SetSafetyEnabled(false);
	//bool gearMoveThreadRunBool = false;
	//std::thread gearMoveThread(moveToGearThreadFunction, &gearMoveThreadRunBool, &pid); //thread not needed yet - might need to be implemented later
	gyro.ZeroYaw();
	float driveX;
	float driveY;
	float driveZ;
	float angle;
	float angleOutput = 0; //pid loop output

	leftProx.SetAutomaticMode(true);
	rightProx.SetAutomaticMode(true);

	while (IsOperatorControl() && IsEnabled())
	{
		angleOutput = 0; //reset output so that if the pid loop isn't being called it's not reserved from the last time it's called
		angle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw();

		if(driveStick.GetPOV() != -1) { //turn to angle 0, 90, 180, 270
			angleOutput = pid.PIDAngle(angle, driveStick.GetPOV()); //call pid loop
		} else {
			pid.resetPID();
		}

		driveX = fabs(driveStick.GetX()) < .05 ? 0.0 : driveStick.GetX(); //deadzones - need to figure out deadzones better (RecycleRush code might have them)
		driveY = fabs(driveStick.GetY()) < .05 ? 0.0 : driveStick.GetY(); //TODO: make getrawaxis w/ numbers instead of getx, gety, getz
		driveZ = fabs(driveStick.GetZ()) < .5 ? 0.0 : driveStick.GetZ() * .33; //Z axis deadzone is huge
		robotDrive.MecanumDrive_Cartesian(driveX, driveY, driveZ + angleOutput);
/*		if (operatorStick.GetRawButton(Constants::runGearMoveThreadButton)) {
			gearMoveThreadRunBool = true;
		} //should be able to reopen the thread after it's closed by the cancel button*/ //thread code - not needed right now
		frc::Wait(0.005); // wait 5ms to avoid hogging CPU cycles
		SmartDashboard::PutNumber("leftProx", leftProx.GetRangeInches());
		SmartDashboard::PutNumber("rightProx", rightProx.GetRangeInches());
		SmartDashboard::PutBoolean("leftIR", !leftIR.Get());
		SmartDashboard::PutBoolean("rightIR", !rightIR.Get());

		SmartDashboard::PutNumber("Angle to gear", aimer.GetAngleToGear());
		SmartDashboard::PutNumber("angleOutput", angleOutput);
		SmartDashboard::PutNumber("Angle", angle);
		SmartDashboard::PutBoolean("Is Rotating", gyro.IsRotating());
		SmartDashboard::PutNumber("Requested Update rate", gyro.GetRequestedUpdateRate());
		SmartDashboard::PutNumber("Actual Update rate", gyro.GetActualUpdateRate());
		SmartDashboard::PutNumber("getPOV", driveStick.GetPOV());

		SmartDashboard::PutNumber("angleToGear", aimer.GetAngleToGear());
	}
	//gearMoveThreadRunBool = false;
	//gearMoveThread.join();
	robotDrive.SetSafetyEnabled(true);
}

void Robot::Autonomous() {
	//This function so far will go through and try to target onto the peg and try to run onto it.
	int failsafe = 0;
	float tryAngle = 0.0;//This is the angle to which the robot will try to aim
	bool isDone;
	switch((int)SmartDashboard::GetNumber("Starting Position", 1))//This gets the starting position from the user
	{
	case 1://Position 1: straight from the middle peg
		tryAngle = 0.0;
		robotDrive.MecanumDrive_Cartesian(0, 1.0, 0.0);//Drive forward?
		isDone = false;
		while (!isDone)//This could be better
		{
			if (gyro.GetDisplacementY() >= 2.0)//~6 ft?
			{
				robotDrive.MecanumDrive_Cartesian(0,0,0);//Stop robot
				isDone = true;//Stop the loop
			}
		}
		break;

	case 2://Position 2: on the left
		tryAngle = 60.0;
		robotDrive.MecanumDrive_Cartesian(0, 1.0, 0.0);
		isDone = false;
		while (!isDone)//Forward 9 feet
		{
			if (gyro.GetDisplacementY() >= 3.0)//9 ft?
			{
				robotDrive.MecanumDrive_Cartesian(0,0,0);
				isDone = true;
			}
		}
		robotDrive.MecanumDrive_Cartesian(0, 0, 0.5);//Turn right?
		isDone = false;
		while (!isDone)
		{
			if (gyro.GetYaw() >= 60)//60 deg?
			{
				robotDrive.MecanumDrive_Cartesian(0,0,0);
				isDone = true;
			}
		}
		break;

	}

	while (!(fabs(aimer.GetAngleToGear()) <= 3.0) && failsafe < 500)//This adjusts the accuracy of the aiming of the robot
	{
		int sign = (aimer.GetAngleToGear() < 0) ? -1 : 1;//Which direction to turn
		robotDrive.MecanumDrive_Cartesian(0, 0, 0.5 * sign);//Turn?
		while (!(gyro.GetYaw() >= tryAngle + sign * 20));//Keep turning to 20 deg away?
		robotDrive.MecanumDrive_Cartesian(0, 1.0, 0);
		while (gyro.GetYaw() + aimer.GetAngleToGear() >= tryAngle);//Keep going striaght until the robot lines up with the peg about
		robotDrive.MecanumDrive_Cartesian(0, 0, 0.5 * sign * -1);
		while (!(gyro.GetYaw() >= tryAngle));//Turn back toward the peg
		frc::Wait(.01);
	}

	//drive until hit while braking
	//drop gear
	//back up quickly
}

START_ROBOT_CLASS(Robot)
