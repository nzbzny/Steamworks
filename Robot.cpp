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

inline float getAverageDistance(const Ultrasonic& leftProx, const Ultrasonic& rightProx);

void Robot::OperatorControl()
{
	robotDrive.SetSafetyEnabled(false);
	//bool gearMoveThreadRunBool = false;
	//std::thread gearMoveThread(moveToGearThreadFunction, &gearMoveThreadRunBool, &pid); //thread not needed yet - might need to be implemented later
	pid.setAngle(SmartDashboard::GetNumber("angle_p", .015), SmartDashboard::GetNumber("angle_i", .001), SmartDashboard::GetNumber("angle_d", .001));
	pid.setY(SmartDashboard::GetNumber("y_p", .025), SmartDashboard::GetNumber("y_i", .001), SmartDashboard::GetNumber("y_d", .001));
	pid.setX(SmartDashboard::GetNumber("x_p", .025), SmartDashboard::GetNumber("x_i", .001), SmartDashboard::GetNumber("x_d", .001));
	gyro.ZeroYaw();
	float driveX;
	float driveY;
	float driveZ;
	float angle;
	float angleOutput = 0; //pid loop output
	float gearAngle = 0;
	float yOutput;
	float xOutput;

	leftProx.SetAutomaticMode(true);
	rightProx.SetAutomaticMode(true);

	while (IsOperatorControl() && IsEnabled())
	{
		SmartDashboard::PutNumber("x_p", 0);
		SmartDashboard::PutNumber("x_i", 0);
		SmartDashboard::PutNumber("x_d", 0);
		angleOutput = 0; //reset output so that if the pid loop isn't being called it's not reserved from the last time it's called
		angle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw();
		yOutput = 0; //reset output
		xOutput = 0;


		bool gyroValid = gyro.IsConnected();
		bool resetButtonPush = driveStick.GetRawButton(11);
		if (resetButtonPush)
		{
			gyro.ResetDisplacement();
			SmartDashboard::PutString("Status Update", "Gyro Displacement Reset");
		}
		bool calibrating = gyro.IsCalibrating();


		if(driveStick.GetPOV() != -1 && gyroValid) { //turn to angle 0, 90, 180, 270
			angleOutput = pid.PIDAngle(angle, driveStick.GetPOV()); //call pid loop
		} else {
			pid.resetPIDAngle();
		}

		if(driveStick.GetRawButton(1)) {
			angleOutput = pid.PIDAngle(angle, gearAngle);
		} else {
			gearAngle = aimer.GetAngleToGear() + angle;
			gearAngle = gearAngle < 360 ? gearAngle : gearAngle - 360;
			gearAngle = gearAngle > 0 ? gearAngle : gearAngle + 360;
		}

		if(driveStick.GetRawButton(2)) {
			yOutput = pid.PIDY(leftProx.GetRangeInches(), rightProx.GetRangeInches());
			xOutput = pid.PIDX(aimer.GetAngleToGear());
		}
		driveX = fabs(driveStick.GetX()) < .05 ? 0.0 : driveStick.GetX(); //deadzones - need to figure out deadzones better (RecycleRush code might have them)
		driveY = fabs(driveStick.GetY()) < .05 ? 0.0 : driveStick.GetY(); //TODO: make getrawaxis w/ numbers instead of getx, gety, getz
		driveZ = fabs(driveStick.GetZ()) < .5 ? 0.0 : driveStick.GetZ() * .33; //Z axis deadzone is huge
		robotDrive.MecanumDrive_Cartesian(driveX + xOutput, driveY + yOutput, driveZ + angleOutput);
/*		if (operatorStick.GetRawButton(Constants::runGearMoveThreadButton)) {
			gearMoveThreadRunBool = true;
		} //should be able to reopen the thread after it's closed by the cancel button*/ //thread code - not needed right now


		frc::Wait(0.005); // wait 5ms to avoid hogging CPU cycles
		SmartDashboard::PutNumber("leftProx", leftProx.GetRangeInches());
		SmartDashboard::PutNumber("rightProx", rightProx.GetRangeInches());
		SmartDashboard::PutBoolean("leftIR", !leftIR.Get());
		SmartDashboard::PutBoolean("rightIR", !rightIR.Get());

		SmartDashboard::PutNumber("Angle to gear (aimer)", aimer.GetAngleToGear());
		SmartDashboard::PutNumber("angleOutput", angleOutput);
		SmartDashboard::PutNumber("Angle", angle);
		SmartDashboard::PutBoolean("Is Rotating", gyro.IsRotating());
		SmartDashboard::PutNumber("Requested Update rate", gyro.GetRequestedUpdateRate());
		SmartDashboard::PutNumber("Actual Update rate", gyro.GetActualUpdateRate());
		SmartDashboard::PutNumber("getPOV", driveStick.GetPOV());
		SmartDashboard::PutNumber("GearAngleCalculated", gearAngle);
		SmartDashboard::PutNumber("angleToGear", aimer.GetAngleToGear());

		SmartDashboard::PutNumber("yOutput", yOutput);
		SmartDashboard::PutNumber("xOutput", xOutput);

		SmartDashboard::PutBoolean("Button 11 pushed", resetButtonPush);
		SmartDashboard::PutBoolean("Button 12 pushed", calibrating);
	}
	//gearMoveThreadRunBool = false;
	//gearMoveThread.join();
	robotDrive.SetSafetyEnabled(true);
}

void Robot::Autonomous() {
	//This function so far will go through and try to target onto the peg and try to run onto it.
	robotDrive.SetSafetyEnabled(false);
	gyro.ResetDisplacement();
	gyro.ZeroYaw();
	int failsafe = 0;
	float tryAngle = 0.0;//This is the angle to which the robot will try to aim
	bool isDone;
	switch((int)SmartDashboard::GetNumber("Starting Position", 1))//This gets the starting position from the user
	{
	case 1://Position 1: straight from the middle peg
		tryAngle = 0.0;
		isDone = false;
		while (!isDone && failsafe < 400 && !IsOperatorControl())//This could be better
		{
			SmartDashboard::PutNumber("xpoZ", gyro.GetDisplacementX());
			SmartDashboard::PutNumber("yPoZ" , gyro.GetDisplacementY());
			float angleChangle = pid.PIDAngle(gyro.GetYaw(), 0.0);
			robotDrive.MecanumDrive_Cartesian(0.0, -0.50, angleChangle);
			if (getAverageDistance(leftProx, rightProx) <= 50)//~6 ft?  lol nope
			{
				robotDrive.MecanumDrive_Cartesian(0.0,0.0,0.0);//Stop robot
				isDone = true;//Stop the loop
			}
			frc::Wait(.01);
			failsafe++;
		}
		failsafe = 0;
		break;

	case 2://Position 2: on the left
		tryAngle = 60.0;
		robotDrive.MecanumDrive_Cartesian(1.0, 0.0, 0.0);
		isDone = false;
		while (!isDone && failsafe < 500 && !IsOperatorControl())//Forward 9 feet
		{
			float angleChangle = pid.PIDAngle(gyro.GetYaw(), 0.0);
			robotDrive.MecanumDrive_Cartesian(0.0, -0.50, angleChangle);
			if (getAverageDistance(leftProx, rightProx) <= 36)//9 ft?
			{
				robotDrive.MecanumDrive_Cartesian(0.0,0.0,0.0);
				isDone = true;
			}
			frc::Wait(.01);
			failsafe++;
		}
		isDone = failsafe = 0;
		while(!isDone && failsafe < 200 && !IsOperatorControl())
		{
			float angleChangle = pid.PIDAngle(gyro.GetYaw(), 60.0);
			robotDrive.MecanumDrive_Cartesian(0.0, 0.0, angleChangle);
			isDone = angleChangle <= 0.05 && angleChangle >= -0.05;
			frc::Wait(.01);
			failsafe++;
		}
		failsafe = 0;
		break;

	}

	float bigFailsafe = 0;

	while (!(fabs(aimer.GetAngleToGear()) <= 3.0) && bigFailsafe < 500 && !IsOperatorControl())//This adjusts the accuracy of the aiming of the robot
	{
		int sign = (aimer.GetAngleToGear() < 0) ? -1 : 1;//Which direction to turn
		while (aimer.GetAngleToGear() < 20 * sign && failsafe < 100 && !IsOperatorControl())
		{
			float angleChangle = pid.PIDAngle(gyro.GetYaw(), aimer.GetAngleToGear() + gyro.GetYaw() + 20 * sign);
			robotDrive.MecanumDrive_Cartesian(0, 0.0, angleChangle);
			frc::Wait(.01);
			failsafe++;
			bigFailsafe++;
		}
		failsafe = 0;
		while (gyro.GetYaw() + aimer.GetAngleToGear() >= tryAngle - 3 && gyro.GetYaw() + aimer.GetAngleToGear() <= tryAngle + 3 && failsafe < 200 && !IsOperatorControl())
		{
			float angleChangle = pid.PIDAngle(gyro.GetYaw(), aimer.GetAngleToGear() + gyro.GetYaw() + 20 * sign);
			robotDrive.MecanumDrive_Cartesian(0, -0.5, angleChangle);
			frc::Wait(.01);
			failsafe++;
		}
		failsafe = 0;
		while (!(gyro.GetYaw() >= tryAngle) && failsafe < 100 && !IsOperatorControl())
		{
			float angleChangle = pid.PIDAngle(gyro.GetYaw(), aimer.GetAngleToGear() + gyro.GetYaw());
			robotDrive.MecanumDrive_Cartesian(0, 0.0, angleChangle);
			frc::Wait(.01);
			failsafe++;
		}
		failsafe = 0;
	}

	while((!leftIR.Get() || !rightIR.Get()) && failsafe < 200 && !IsOperatorControl())
	{
		float angleChangle = pid.PIDAngle(gyro.GetYaw(), aimer.GetAngleToGear() + gyro.GetYaw());
		float driveSpeed = 0.5 - ((getAverageDistance(leftProx, rightProx) / 80) - 0.5);
		robotDrive.MecanumDrive_Cartesian(0, -driveSpeed, angleChangle);
		frc::Wait(.01);
		failsafe++;
	}

	//drop gear


}

inline float getAverageDistance(const Ultrasonic& leftProx, const Ultrasonic& rightProx)
{
	return ((float)leftProx.GetRangeInches() + (float)rightProx.GetRangeInches()) / 2.0;
}

START_ROBOT_CLASS(Robot)
