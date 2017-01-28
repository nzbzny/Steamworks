#include "Robot.h"
#include "WPILib.h"

/*void moveToGearThreadFunction(bool *keepRunning, PIDLoop *pid) {
	while (*keepRunning == true) {
		if (pid->runPID() == 1) {
			*keepRunning = false;
		}
	}
}*/

static float scaleJoysticks(float power, float dead, float max, int degree) {
	if (degree < 0) {	// make sure degree is positive
		degree = 1;
	}
	if (degree % 2 == 0) {	// make sure degree is odd
		degree++;
	}
	if (fabs(power) < dead) {	// if joystick input is in dead zone, return 0
		return 0;
	}
	else if  (power > 0) {	// if it is outside of the dead zone, then the output is a function of specified degree centered at the end of the dead zone
		return (max * pow(power - dead, degree) / pow(1 - dead, degree));
	}
	else {
		return (max * pow(power + dead, degree) / pow(1 - dead, degree));
	}
}

Robot::Robot() :
		/*frontLeftMotor(Constants::frontLeftDriveChannel),
		rearLeftMotor(Constants::rearLeftDriveChannel),
		frontRightMotor(Constants::frontRightDriveChannel),
		rearRightMotor(Constants::rearRightDriveChannel),*/
		robotDrive(Constants::frontLeftDriveChannel, Constants::rearLeftDriveChannel, Constants::frontRightDriveChannel, Constants::rearRightDriveChannel),
		driveStick(Constants::driveStickChannel),
		operatorStick(Constants::operatorStickChannel),
		gyro(I2C::Port::kMXP, 200),
		pid(),
		aimer(),
		leftProx(1, 0),
		rightProx(3, 2),
		leftIR(5),
		rightIR(4),
		gear(Constants::gearReleaseInSole, Constants::gearReleaseOutSole),
		shooter(Constants::rotatorChannel, Constants::shooterChannel, Constants::agitatorChannel),
		compressor(Constants::compressorPin)

{
	robotDrive.SetExpiration(0.1);
	robotDrive.SetInvertedMotor(RobotDrive::kFrontLeftMotor, false);
	robotDrive.SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
	robotDrive.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
	robotDrive.SetInvertedMotor(RobotDrive::kRearRightMotor, true);
}

void Robot::RobotInit() {
	CameraServer::GetInstance()->StartAutomaticCapture();
	SmartDashboard::PutNumber("cameraBetweeness", 15.75);
}
/**
 * Runs the motors with Mecanum drive.
 */

//inline float getAverageDistance(const Ultrasonic& leftProx, const Ultrasonic& rightProx);

void Robot::OperatorControl()
{
	robotDrive.SetSafetyEnabled(false);
	//bool gearMoveThreadRunBool = false;
	//std::thread gearMoveThread(moveToGearThreadFunction, &gearMoveThreadRunBool, &pid); //thread not needed yet - might need to be implemented later
	pid.setAngle(SmartDashboard::GetNumber("angle_p", Constants::angle_p_default), SmartDashboard::GetNumber("angle_i", Constants::angle_i_default), SmartDashboard::GetNumber("angle_d", Constants::angle_d_default));
	pid.setY(SmartDashboard::GetNumber("y_p", Constants::y_p_default), SmartDashboard::GetNumber("y_i", Constants::y_i_default), SmartDashboard::GetNumber("y_d", Constants::y_d_default));
	pid.setX(SmartDashboard::GetNumber("x_p", Constants::x_p_default), SmartDashboard::GetNumber("x_i", Constants::x_i_default), SmartDashboard::GetNumber("x_d", Constants::x_d_default));
	gyro.ZeroYaw();
	float driveX;
	float driveY;
	float driveZ;
	float angle;
	float angleOutput = 0; //pid loop output
	float gearAngle = 0;
	float yOutput;
	float xOutput;
	float voltage = 0; //testing data for battery voltage
	bool gyroValid;
	bool resetButtonPush;
	bool calibrating;
	float currentAngle = 0;

	leftProx.SetAutomaticMode(true);
	rightProx.SetAutomaticMode(true);
	shooter.enable();
	compressor.Start();

	while (IsOperatorControl() && IsEnabled())
	{
		angleOutput = 0; //reset output so that if the pid loop isn't being called it's not reserved from the last time it's called
		angle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw();
		yOutput = 0; //reset output
		xOutput = 0;
		voltage = DriverStation::GetInstance().GetBatteryVoltage();


		gyroValid = gyro.IsConnected();
		resetButtonPush = driveStick.GetRawButton(11);
		if (resetButtonPush)
		{
			gyro.ResetDisplacement();
			SmartDashboard::PutString("Status Update", "Gyro Displacement Reset");
		}
		calibrating = gyro.IsCalibrating();

		if(driveStick.GetRawButton(4)) {
			angleOutput = pid.PIDAngle(angle, currentAngle);
			yOutput = SmartDashboard::GetNumber("tempMovePower", 0.0);
			Wait(.25);
			robotDrive.MecanumDrive_Cartesian(0.0, yOutput, 0.0);
			Wait(.5);
		} else {
			currentAngle = angle;
		}

		if(driveStick.GetRawButton(Constants::gearReleaseButton)) { //TODO: may need to swap values
			gear.setBottom(true);
		} else {
			gear.setBottom(false);
		}

		if(driveStick.GetPOV() != -1 && gyroValid && (driveStick.GetPOV() % 90 == 0)) { //turn to angle 0, 90, 180, 270
			angleOutput = pid.PIDAngle(angle, driveStick.GetPOV()); //call pid loop
		} else {
			pid.resetPIDAngle(); //if loop is done reset values
		}

		if(driveStick.GetRawButton(Constants::turnToGearButton)) { //turn to gear
			angleOutput = pid.PIDAngle(angle, gearAngle);
		} else {
			gearAngle = aimer.twoCameraAngleFilter() + angle; //reset absolute angle gear is at
			gearAngle = gearAngle < 360 ? gearAngle : gearAngle - 360; //scaling
			gearAngle = gearAngle > 0 ? gearAngle : gearAngle + 360;
			pid.resetPIDAngle(); //if loop is done reset values
		}

		if(driveStick.GetRawButton(Constants::moveToGearButton)) { //pid move
			angleOutput = pid.PIDAngle(angle, 0); //TODO: get angle via function (closest angle of the 3 options?)
			if (angleOutput == 0) {
				xOutput = pid.ultrasonicFilter(leftProx.GetRangeInches(), rightProx.GetRangeInches()) > 45 ? /*pid.PIDX(aimer.GetLeftAngleToGear()) : 0.0;*/ -pid.PIDX(aimer.twoCameraAngleFilter()) : 0.0;
				yOutput = (xOutput == 0.0) ? pid.PIDY(leftProx.GetRangeInches(), rightProx.GetRangeInches()) : 0.0;
			}
		} else { //if loop is done reset values
			pid.resetPIDX();
			pid.resetPIDY();
		}

		//scaling for the joystick deadzones
		driveX = driveStick.GetRawAxis(Constants::driveXAxis);
		driveY = driveStick.GetRawAxis(Constants::driveYAxis);
		driveZ = driveStick.GetRawAxis(Constants::driveZAxis);

		driveX = scaleJoysticks(driveX, Constants::driveXDeadZone, Constants::driveXMax * (.5 - (driveStick.GetRawAxis(Constants::driveThrottleAxis) / 2)), Constants::driveXDegree);
		driveY = scaleJoysticks(driveY, Constants::driveYDeadZone, Constants::driveYMax * (.5 - (driveStick.GetRawAxis(Constants::driveThrottleAxis) / 2)), Constants::driveYDegree);
		driveZ = scaleJoysticks(driveZ, Constants::driveZDeadZone, Constants::driveZMax * (.5 - (driveStick.GetRawAxis(Constants::driveThrottleAxis) / 2)), Constants::driveZDegree);

	if (driveStick.GetRawButton(Constants::driveOneAxisButton)) { //drive only one axis
		if (fabs(driveX) > fabs(driveY) && fabs(driveX) > fabs(driveZ)) { //if X is greater than Y and Z, then it will only go in the direction of X
			driveY = 0;
			driveZ = 0;
		}
		else if (fabs(driveY) > fabs(driveX) && fabs(driveY) > fabs(driveZ)) { //if Y is greater than X and Z, then it will only go in the direction of Y
			driveX = 0;
			driveZ = 0;
		}
		else { //if Z is greater than X and Y, then it will only go in the direction of Z
			driveX = 0;
			driveY = 0;
		}
	}

	driveX = fabs(driveX + xOutput) > 1 ? std::copysign(1, driveX + xOutput) : driveX + xOutput;
	driveY = fabs(driveY + yOutput) > 1 ? std::copysign(1, driveY + yOutput) : driveY + yOutput;
	driveZ = fabs(driveZ + angleOutput) > 1 ? std::copysign(1, driveZ + angleOutput) : driveZ + angleOutput;

	robotDrive.MecanumDrive_Cartesian(driveX + xOutput, driveY + yOutput, driveZ + angleOutput);
/*		if (operatorStick.GetRawButton(Constants::runGearMoveThreadButton)) {
			gearMoveThreadRunBool = true;
		} //should be able to reopen the thread after it's closed by the cancel button*/ //thread code - not needed right now


		frc::Wait(0.01); // wait 5ms to avoid hogging CPU cycles TODO: change back to .005 if necessary
		SmartDashboard::PutNumber("leftProx", leftProx.GetRangeInches());
		SmartDashboard::PutNumber("rightProx", rightProx.GetRangeInches());
		SmartDashboard::PutBoolean("leftIR", !leftIR.Get());
		SmartDashboard::PutBoolean("rightIR", !rightIR.Get());

		SmartDashboard::PutNumber("Angle to gear (aimer)", aimer.twoCameraAngleFilter());
		SmartDashboard::PutNumber("angleOutput", angleOutput);
		SmartDashboard::PutNumber("Angle", angle);
		SmartDashboard::PutBoolean("Is Rotating", gyro.IsRotating());
		SmartDashboard::PutNumber("Requested Update rate", gyro.GetRequestedUpdateRate());
		SmartDashboard::PutNumber("Actual Update rate", gyro.GetActualUpdateRate());
		SmartDashboard::PutNumber("getPOV", driveStick.GetPOV());
		SmartDashboard::PutNumber("GearAngleCalculated", gearAngle);
		SmartDashboard::PutNumber("Absolute Angle To Gear", gearAngle);

		SmartDashboard::PutNumber("yOutput", yOutput);
		SmartDashboard::PutNumber("xOutput", xOutput);

		SmartDashboard::PutBoolean("resetButtonPushed", resetButtonPush);
		SmartDashboard::PutBoolean("calibrateButtonPushed", calibrating);

		SmartDashboard::PutNumber("voltage", voltage);
		SmartDashboard::PutNumber("Ultrasonic Filter", pid.ultrasonicFilter(leftProx.GetRangeInches(), rightProx.GetRangeInches()));
		SmartDashboard::PutNumber("leftAngleToGear", aimer.GetLeftAngleToGear());
		SmartDashboard::PutNumber("rightAngleToGear", aimer.GetRightAngleToGear());
		SmartDashboard::PutNumber("distancinator", aimer.Distancinator());

		SmartDashboard::PutNumber("native yaw", gyro.GetYaw());


	}
	//gearMoveThreadRunBool = false;
	//gearMoveThread.join();
	robotDrive.SetSafetyEnabled(true);
	shooter.disable();
	compressor.Stop();
}

void Robot::Autonomous() {
	//This function so far will go through and try to target onto the peg and try to run onto it.
	robotDrive.SetSafetyEnabled(false);
	gyro.ResetDisplacement();
	gyro.ZeroYaw();
	int failsafe = 0;
	float tryAngle = 0.0;//This is the angle to which the robot will try to aim
	bool isDone;
	switch(1)//(int)SmartDashboard::GetNumber("Starting Position", 1))//This gets the starting position from the user
	{
	case 1://Position 1: straight from the middle peg
		tryAngle = 0.0;
		isDone = false;
		while (!isDone && failsafe < 400 && !IsOperatorControl())//This could be better
		{
			SmartDashboard::PutNumber("xpoZ", gyro.GetDisplacementX());
			SmartDashboard::PutNumber("yPoZ" , gyro.GetDisplacementY());
			float angleChangle = pid.PIDAngle(gyro.GetYaw(), 0.0);
			float xOutput = pid.PIDX(aimer.twoCameraAngleFilter());
			float yOutput = pid.PIDY(leftProx.GetRangeInches(), rightProx.GetRangeInches());
			robotDrive.MecanumDrive_Cartesian(xOutput, yOutput, angleChangle);
			if (pid.ultrasonicFilter(leftProx.GetRangeInches(), rightProx.GetRangeInches()) <= 50)//~6 ft?  lol nope
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
		while (!isDone && failsafe < 110 && !IsOperatorControl())//Forward 9 feet
		{
			float currentAngle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw();
			float angleChangle = pid.PIDAngle(currentAngle, 0);
			robotDrive.MecanumDrive_Cartesian(0.0, -0.6, angleChangle);
			if (pid.ultrasonicFilter(leftProx.GetRangeInches(), rightProx.GetRangeInches()) <= 36)//9 ft?
			{
				robotDrive.MecanumDrive_Cartesian(0.0,0.0,0.0);
				isDone = true;
			}
			frc::Wait(.01);
			failsafe++;
		}
		isDone = false;
		failsafe = 0;
		while(!isDone && failsafe < 200 && !IsOperatorControl())
		{
			float currentAngle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw();
			float angleChangle = pid.PIDAngle(currentAngle, 300);
			float xOutput = 0;//pid.PIDX(aimer.twoCameraAngleFilter());
			float yOutput = 0;//pid.PIDY(leftProx.GetRangeInches(), rightProx.GetRangeInches());
			robotDrive.MecanumDrive_Cartesian(xOutput, yOutput, angleChangle);
			isDone = angleChangle <= 0.05 && angleChangle >= -0.05;
			frc::Wait(.01);
			failsafe++;
		}
		failsafe = 0;
		break;

	}

	float bigFailsafe = 0;

	while (!(fabs(aimer.twoCameraAngleFilter()) <= 3.0) && bigFailsafe < 500 && !IsOperatorControl())//This adjusts the accuracy of the aiming of the robot
	{
		int sign = (aimer.twoCameraAngleFilter() < 0) ? -1 : 1;//Which direction to turn
		while (aimer.twoCameraAngleFilter() < 20 * sign && failsafe < 100 && !IsOperatorControl())
		{
			float angleChangle = pid.PIDAngle(gyro.GetYaw(), aimer.twoCameraAngleFilter() + gyro.GetYaw() + 20 * sign);
			robotDrive.MecanumDrive_Cartesian(0, 0.0, angleChangle);
			SmartDashboard::PutNumber("Angle to Gear", aimer.twoCameraAngleFilter());
			frc::Wait(.01);
			failsafe++;
			bigFailsafe++;
		}
		failsafe = 0;
		while (gyro.GetYaw() + aimer.twoCameraAngleFilter() >= tryAngle - 3 && gyro.GetYaw() + aimer.twoCameraAngleFilter() <= tryAngle + 3 && failsafe < 200 && !IsOperatorControl())
		{
			float angleChangle = pid.PIDAngle(gyro.GetYaw(), aimer.twoCameraAngleFilter() + gyro.GetYaw() + 20 * sign);
			robotDrive.MecanumDrive_Cartesian(0, -0.5, angleChangle);
			SmartDashboard::PutNumber("Angle to Gear", aimer.twoCameraAngleFilter());
			frc::Wait(.01);
			failsafe++;
		}
		failsafe = 0;
		while (!(gyro.GetYaw() >= tryAngle) && failsafe < 100 && !IsOperatorControl())
		{
			float angleChangle = pid.PIDAngle(gyro.GetYaw(), aimer.twoCameraAngleFilter() + gyro.GetYaw());
			robotDrive.MecanumDrive_Cartesian(0, 0.0, angleChangle);
			SmartDashboard::PutNumber("Angle to Gear", aimer.twoCameraAngleFilter());
			frc::Wait(.01);
			failsafe++;
		}
		failsafe = 0;
	}

	while(pid.ultrasonicFilter(leftProx.GetRangeInches(), rightProx.GetRangeInches()) > 12 && failsafe < 200 && !IsOperatorControl())
	{
		float angleChangle = pid.PIDAngle(gyro.GetYaw(), aimer.twoCameraAngleFilter() + gyro.GetYaw());
		float driveSpeed = 0.5 - ((pid.ultrasonicFilter(leftProx.GetRangeInches(), rightProx.GetRangeInches()) / 80) - 0.5);
		robotDrive.MecanumDrive_Cartesian(0, -driveSpeed, angleChangle);
		SmartDashboard::PutNumber("Angle to Gear", aimer.twoCameraAngleFilter());
		SmartDashboard::PutNumber("leftProx", leftProx.GetRangeInches());
		SmartDashboard::PutNumber("rightProx", rightProx.GetRangeInches());
		SmartDashboard::PutNumber("leftIR", leftIR.Get());
		SmartDashboard::PutNumber("rightIR", rightIR.Get());\
		SmartDashboard::PutNumber("Ultrasonic Filter", pid.ultrasonicFilter(leftProx.GetRangeInches(), rightProx.GetRangeInches()));
		frc::Wait(.01);
		failsafe++;
	}

	//drop gear


}

/*inline float getAverageDistance(const Ultrasonic& leftProx, const Ultrasonic& rightProx)
{
	return ((float)leftProx.GetRangeInches() + (float)rightProx.GetRangeInches()) / 2.0;
}*/

START_ROBOT_CLASS(Robot)
