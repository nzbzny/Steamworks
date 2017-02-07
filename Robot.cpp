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
		gyro(I2C::Port::kMXP, 100),
		pid(),
		aimer(),
		leftProx(1, 0),
		rightProx(3, 2),
		leftIR(5),
		rightIR(4),
		gear(Constants::gearReleaseInSole, Constants::gearReleaseOutSole),
		shooter(Constants::rotatorChannel, Constants::shooterChannel, Constants::agitatorChannel),
		compressor(Constants::compressorPin),
		filter()

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
	pid.setAngle(SmartDashboard::GetNumber("angle_p", Constants::angle_p_default), SmartDashboard::GetNumber("angle_i", Constants::angle_i_default), SmartDashboard::GetNumber("angle_d", Constants::angle_d_default));
	pid.setY(SmartDashboard::GetNumber("y_p", Constants::y_p_default), SmartDashboard::GetNumber("y_i", Constants::y_i_default), SmartDashboard::GetNumber("y_d", Constants::y_d_default));
	pid.setX(SmartDashboard::GetNumber("x_p", Constants::x_p_default), SmartDashboard::GetNumber("x_i", Constants::x_i_default), SmartDashboard::GetNumber("x_d", Constants::x_d_default));
	gyro.ZeroYaw();
	gyro.Reset();
	gyro.ResetDisplacement();

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
	bool calibrating;

	bool shooterAutoAngleButtonPressed = false;
	bool shooterShootButtonPressed = false;
	bool shooting = false;
	float shooterSpeed = 0.0;
	bool shooterAngleReached = true;

	bool gearButtonPressed = false;
	int gearOpenCounter = 0;
	int gearOpenMaxCount = 1000;

	bool oneAxisButtonPressed = false;
	float oneAxisDesiredAngle = 0.0;

	//ultrasonics
	float leftUltrasonic = leftProx.GetRangeInches();
	float rightUltrasonic = rightProx.GetRangeInches();

	//init functions
	filter.initializeLastUltrasonics(leftUltrasonic, rightUltrasonic);
	filter.initializePredictedValue(leftUltrasonic, rightUltrasonic);
	leftProx.SetAutomaticMode(true);
	rightProx.SetAutomaticMode(true);
	shooter.enable();
	compressor.Start();


	//  dgTest stuff
	//
//	double yTargetDistance = -9999.0;
//	bool inYTargetMode = false;
//	double angleTarget = -9999.0;
//	bool inAngleTargetMode = false;
//	double xTargetDistance = -9999.0;
//	bool inXTargetMode = false;
//	double lastLeftProxValue = 0;
//	double lastLeftProxTime = 0;
//	double lastRightProxValue = 0;
//	double lastrightProxTime = 0;
//	double lastCameraAngleToGear = 0;
//	double lastCameraAngleToGearTime = 0;
//	Timer dgTimer = new Timer();
//	double lastLoopTimeValue = dgTimer.Get();
//	std::tuple(double, double, double) positionHistory[50];
	//
	//


	while (IsOperatorControl() && IsEnabled())
	{

//		if(driveStick.GetRawButton(5)){
//
//			//  DG code to  test the alternative predictive drive method
//			if(!inYTargetMode){
//
//				inYTargetMode = true;
//				yTargetDistance = 12;
//
//			}
//
//			if (!inAngleTargetMode){
//				inAngleTargetMode = true;
//				angleTarget = 0;
//
//			}
//
//			//
//			// get the distance
//			leftUltrasonic = leftProx.GetRangeInches();
//			rightUltrasonic = rightProx.GetRangeInches();
//			SmartDashboard::PutNumber("leftProx", leftUltrasonic);
//			SmartDashboard::PutNumber("rightProx", rightUltrasonic);
//			double ultrasonicAverage = (leftUltrasonic + rightUltrasonic )/2;
//
//
//			// get the target angle
//			float angle = aimer.GetAngleToGear();
//			//calculate the x offset
//			double xOffset = ultrasonicAverage * sin(angle*PI/180);
//			if(!inXTargetMode){
//				xTargetDistance = xOffset;
//				inXTargetMode = true;
//			}
//
//
//
//		}else{
//			inXTargetMode = false;
//			inYTargetMode = false;
//			inAngleTargetMode = false;
//		}


		if (driveStick.GetRawButton(11)) { //TODO: delete test code
			float kalman = filter.kalmanFilter(leftUltrasonic, rightUltrasonic, -.50);
			while (kalman > 50) {
				SmartDashboard::PutBoolean("In Kalman Test Loop", true);
				leftUltrasonic = leftProx.GetRangeInches();
				rightUltrasonic = rightProx.GetRangeInches();
				SmartDashboard::PutNumber("leftProx", leftUltrasonic);
				SmartDashboard::PutNumber("rightProx", rightUltrasonic);
				SmartDashboard::PutNumber("Ultrasonic Filter", filter.ultrasonicFilter(leftUltrasonic, rightUltrasonic));
				robotDrive.MecanumDrive_Cartesian(0.0, -.50, 0.0);
				kalman = filter.kalmanFilter(leftUltrasonic, rightUltrasonic, -.50);
				SmartDashboard::PutNumber("Ultrasonic Kalman Filter", kalman);
				Wait(.01);
			}
			SmartDashboard::PutBoolean("In Kalman Test Loop", false);
			robotDrive.MecanumDrive_Cartesian(0.0, 0.0, 0.0);
		}



		/*
		 *
		 * BASIC CHECKS
		 *
		 */

		 gyroValid = gyro.IsConnected();
		 calibrating = gyro.IsCalibrating();
		 voltage = DriverStation::GetInstance().GetBatteryVoltage();
		 angleOutput = 0; //reset output so that if the pid loop isn't being called it's not reserved from the last time it's called
		 angle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw();
		 yOutput = 0; //reset output
		 xOutput = 0;
		 leftUltrasonic = leftProx.GetRangeInches();
		 rightUltrasonic = rightProx.GetRangeInches();
		 if (driveStick.GetRawButton(Constants::cancelAllButton)) {
			 shooterAutoAngleButtonPressed = true;
			 shooterShootButtonPressed = true;
			 shooting = false;
			 shooterSpeed = 0.0;
			 shooterAngleReached = true;
			 gearButtonPressed = true;
			 gearOpenCounter = 1000;
		 }

		/*
		 *
		 * END BASIC CHECKS
		 *
		 */

		/*
		 *
		 * SHOOTER CODE
		 *
		 */

		if (driveStick.GetRawButton(Constants::shooterAutoAngleButton) && !shooterAutoAngleButtonPressed && !shooterAngleReached) { //if the shooter has been started and the button was let go and then pressed
			shooterAngleReached = true; //cancel shooter auto aim
		}
		if (driveStick.GetRawButton(Constants::shooterAutoAngleButton) && !shooterAutoAngleButtonPressed && shooterAngleReached)  { //if the shooter has not been started and the button was let go and then pressed
			shooterAngleReached = shooter.setAngle(0); //TODO: need to get angle from tj's vision code
			shooterAutoAngleButtonPressed = true; //set button pressed to true so that holding the button for more than 10ms (loop time) doesn't activate the loop above and cancel it
		}
		if (!driveStick.GetRawButton(Constants::shooterAutoAngleButton)) { //if the shooter button has been let go
			shooterAutoAngleButtonPressed = false; //set to false so it can be pressed again
		}
		if (!shooterAngleReached) { //if the shooter angle hasn't yet been reached
			shooterAngleReached = shooter.setAngle(0); //TODO: get angle from tj's vision code
		}
		if (driveStick.GetRawButton(Constants::shooterShootButton) && !shooterShootButtonPressed && !shooting) { //shoot
			//TODO: get shooter speed
			shooter.shoot(shooterSpeed); //shoot at the speed
			shooterShootButtonPressed = true; //button is pressed so it doesn't immediately cancel
			shooting = true; //set shooting to true so it knows next time the button is pressed to cancel
		} else if (driveStick.GetRawButton(Constants::shooterShootButton) && !shooterShootButtonPressed && shooting) { //cancel
			shooter.stop(); //turn off shooter
			shooterShootButtonPressed = true; //so it doesn't immediately start shooting again
			shooting = false; //so it knows what loop to go into
		}
		if (!driveStick.GetRawButton(Constants::shooterShootButton)) { //when button is let go
			shooterShootButtonPressed = false; //let shooter change state
		}

		/*
		 *
		 * END SHOOTER CODE
		 *
		 */



		/*
		 *
		 * GEAR CODE
		 *
		 */

		if(driveStick.GetRawButton(Constants::gearReleaseButton) && !gearButtonPressed) { //TODO: may need to swap values
			gear.setBottom(!gear.getBottom()); //set to the opposite of what it currently is
			gearButtonPressed = true; //button still pressed = true
		} else if (!driveStick.GetRawButton(Constants::gearReleaseButton)) { //button not pressed - so it doesn't keep flipping between open and closed
			gearButtonPressed = false; //button not pressed
		}
		if (gear.getBottom()) { //TODO: may need to make a not
			gearOpenCounter++; //timer for how long the gear is open before the operator just forgot to close it
		} else {
			gearOpenCounter = 0; //reset gear open counter
		}
		if (gearOpenCounter > gearOpenMaxCount) { //if it's been open for more than 10 seconds
			gear.setBottom(!gear.getBottom()); //set to closed
			gearOpenCounter = 0;
		}

		/*
		 *
		 * END GEAR CODE
		 *
		 */


		/*
		 *
		 * PID CODE
		 *
		 */

		if(driveStick.GetPOV() != -1 && gyroValid) { //turn to angle 0, 90, 180, 270
			angleOutput = pid.PIDAngle(angle, driveStick.GetPOV()); //call pid loop
		} else {
			pid.resetPIDAngle(); //if loop is done reset values
		}

		if(driveStick.GetRawButton(Constants::turnToGearButton)) { //turn to gear
			angleOutput = pid.PIDAngle(angle, gearAngle);
		} else {
			gearAngle = aimer.TwoCameraAngleFilter() + angle; //reset absolute angle gear is at
			gearAngle = gearAngle < 360 ? gearAngle : gearAngle - 360; //scaling
			gearAngle = gearAngle > 0 ? gearAngle : gearAngle + 360;
			pid.resetPIDAngle(); //if loop is done reset values
		}

		if(driveStick.GetRawButton(Constants::moveToGearButton)) { //pid move
			angleOutput = pid.PIDAngle(angle, 0); //TODO: get angle via function (closest angle of the 3 options (0, 60, -60, find which is closest to current angle)?)
			if (angleOutput == 0) {
				xOutput = filter.ultrasonicFilter(leftUltrasonic, rightUltrasonic) > 45 ? pid.PIDX(aimer.TwoCameraAngleFilter()) : 0.0; //if done turning move x
				yOutput = (xOutput < .01) ? pid.PIDY(leftUltrasonic, rightUltrasonic) : 0.0; //if done moving x move y
			}
		} else { //if loop is done reset values
			pid.resetPIDX();
			pid.resetPIDY();
		}

		/*
		 *
		 * END PID CODE
		 *
		 */

		/*
		 *
		 * JOYSTICK CODE
		 *
		 */

		//scaling for the joystick deadzones
		driveX = driveStick.GetRawAxis(Constants::driveXAxis);
		driveY = driveStick.GetRawAxis(Constants::driveYAxis);
		driveZ = driveStick.GetRawAxis(Constants::driveZAxis);

		driveX = scaleJoysticks(driveX, Constants::driveXDeadZone, Constants::driveXMax * (.5 - (driveStick.GetRawAxis(Constants::driveThrottleAxis) / 2)), Constants::driveXDegree);
		driveY = scaleJoysticks(driveY, Constants::driveYDeadZone, Constants::driveYMax * (.5 - (driveStick.GetRawAxis(Constants::driveThrottleAxis) / 2)), Constants::driveYDegree);
		driveZ = scaleJoysticks(driveZ, Constants::driveZDeadZone, Constants::driveZMax * (.5 - (driveStick.GetRawAxis(Constants::driveThrottleAxis) / 2)), Constants::driveZDegree);

		if (driveStick.GetRawButton(Constants::driveOneAxisButton)) { //drive only one axis
			if (!oneAxisButtonPressed) { //if not in the middle of being held down
				oneAxisButtonPressed = true; //being held down now - don't reset desired angle
				oneAxisDesiredAngle = angle; //set desired angle
			}
			if (fabs(driveX) > fabs(driveY) && fabs(driveX) > fabs(driveZ)) { //if X is greater than Y and Z, then it will only go in the direction of X
				angleOutput = pid.PIDAngle(angle, oneAxisDesiredAngle); //stay straight
				driveY = 0;
				driveZ = 0;
			}
			else if (fabs(driveY) > fabs(driveX) && fabs(driveY) > fabs(driveZ)) { //if Y is greater than X and Z, then it will only go in the direction of Y
				angleOutput = pid.PIDAngle(angle, oneAxisDesiredAngle); //stay straight
				driveX = 0;
				driveZ = 0;
			}
			else { //if Z is greater than X and Y, then it will only go in the direction of Z
				driveX = 0;
				driveY = 0;
			}
		} else {
			oneAxisButtonPressed = false; //lets you reset the straight facing angle when you let go of the button
		}

		driveX = fabs(driveX + xOutput) > 1 ? std::copysign(1, driveX + xOutput) : driveX + xOutput; //if driving and pid'ing (pls dont) maxes you out at 1 so the motors move
		driveY = fabs(driveY + yOutput) > 1 ? std::copysign(1, driveY + yOutput) : driveY + yOutput; //if driving and pid'ing (pls dont) maxes you out at 1 so the motors move
		driveZ = fabs(driveZ + angleOutput) > 1 ? std::copysign(1, driveZ + angleOutput) : driveZ + angleOutput; //if driving and pid'ing (pls dont) maxes you out at 1 so the motors move


		/*
		 *
		 * END JOYSTICK CODE
		 *
		 */

		robotDrive.MecanumDrive_Cartesian(driveX, driveY, driveZ); //drive


		/*
		 *
		 * SMART DASHBOARD
		 *
		 */

		SmartDashboard::PutNumber("leftProx", leftUltrasonic);
		SmartDashboard::PutNumber("rightProx", rightUltrasonic);
		SmartDashboard::PutBoolean("leftIR", leftIR.get());
		SmartDashboard::PutBoolean("rightIR", rightIR.get());
		SmartDashboard::PutNumber("angleOutput", angleOutput);
		SmartDashboard::PutNumber("Angle", angle);
		SmartDashboard::PutBoolean("Is Rotating", gyro.IsRotating());
		SmartDashboard::PutNumber("Requested Update rate", gyro.GetRequestedUpdateRate());
		SmartDashboard::PutNumber("Actual Update rate", gyro.GetActualUpdateRate());
		SmartDashboard::PutNumber("getPOV", driveStick.GetPOV());
		SmartDashboard::PutNumber("GearAngleCalculated", gearAngle);
		SmartDashboard::PutNumber("TwoCameraAngleFilter", aimer.TwoCameraAngleFilter());
		SmartDashboard::PutNumber("leftAngleToGear", aimer.GetLeftAngleToGear());
		SmartDashboard::PutNumber("rightAngleToGear", aimer.GetRightAngleToGear());
		SmartDashboard::PutNumber("yOutput", yOutput);
		SmartDashboard::PutNumber("xOutput", xOutput);
//		SmartDashboard::PutNumberArray("leftAngleArray", aimer.GetLeftAngleArray());
//		SmartDashboard::PutNumberArray("rightAngleArray", aimer.GetRightAngleArray());

		SmartDashboard::PutBoolean("calibrateButtonPushed", calibrating);

		SmartDashboard::PutNumber("voltage", voltage);
		SmartDashboard::PutNumber("Ultrasonic Filter", filter.ultrasonicFilter(leftUltrasonic, rightUltrasonic));

		SmartDashboard::PutNumber("AngleToGear", aimer.GetAngleToGear());


		SmartDashboard::PutNumber("xDisp", gyro.GetDisplacementX());
		SmartDashboard::PutNumber("yDisp", gyro.GetDisplacementY());
		SmartDashboard::PutNumber("xVel", gyro.GetVelocityX());
		SmartDashboard::PutNumber("yVel", gyro.GetVelocityY());
		SmartDashboard::PutNumber("xAccel", gyro.GetRawAccelX());
		SmartDashboard::PutNumber("yAccel", gyro.GetRawAccelY());
		SmartDashboard::PutBoolean("IsCalibrating", gyro.IsCalibrating());
		SmartDashboard::PutNumber("Ultrasonic Kalman Filter", filter.kalmanFilter(leftUltrasonic, rightUltrasonic, driveY));

		/*
		 *
		 * END SMART DASHBOARD
		 *
		 */

		frc::Wait(0.01); // wait 5ms to avoid hogging CPU cycles TODO: change back to .005 if necessary
		gyro.UpdateDisplacement(gyro.GetRawAccelX(), gyro.GetRawAccelY(), gyro.GetActualUpdateRate(), gyro.IsMoving()); //update gyro displacement

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
	float desiredAngle = 0.0;//This is the angle to which the robot will try to aim
	float angleChangle = 0.0;
	float yOutput = 0.0;
	float currentAngle = 0.0;
	float leftUltrasonic = leftProx.GetRangeInches();
	float rightUltrasonic = rightProx.GetRangeInches();
	bool isDone;
	switch(1)//(int)SmartDashboard::GetNumber("Starting Position", 1))//This gets the starting position from the user
	{
	case 1://Position 1: straight from the middle peg
		desiredAngle = 0.0;
		isDone = false;
		while (!isDone && failsafe < 400 && !IsOperatorControl()) {
			currentAngle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw();
			angleChangle = pid.PIDAngle(currentAngle, desiredAngle); //set angle output
			yOutput = pid.PIDY(leftUltrasonic, rightUltrasonic); //TODO: change when we get the kalman filters on
			robotDrive.MecanumDrive_Cartesian(0.0, yOutput, angleChangle); //drive forward while staying straight
			if (filter.ultrasonicFilter(leftUltrasonic, rightUltrasonic) < 12) { //TODO: change when we get the kalman filters on
				isDone = true; //stop the loop
				failsafe = 400;
			}
			frc::Wait(.01); //wait to avoid hogging cpu cycles
			failsafe++; //increment failsafe variable
		}
		robotDrive.MecanumDrive_Cartesian(0.0, 0.0, 0.0); //stop robot
		failsafe = 0; //reset failsafe for use later
		break;
	case 2://Position 2: on the left
		desiredAngle = 60;
		isDone = false;
		while (!isDone && failsafe < 500 && !IsOperatorControl()) { //TODO: change failsafe < 500 to be the calculated value to go the desired distance
			currentAngle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw(); //current angle
			angleChangle = pid.PIDAngle(currentAngle, 0.0); //drive straight
			robotDrive.MecanumDrive_Cartesian(0.0, -.5, angleChangle); //drive straight for a bit
			frc::Wait(.01);
			failsafe++;
		}
		robotDrive.MecanumDrive_Cartesian(0.0, 0.0, 0.0);
		failsafe = 0;
		while (!isDone && failsafe < 200 && !IsOperatorControl()) {
			currentAngle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw(); //current angle
			leftUltrasonic = leftProx.GetRangeInches();
			rightUltrasonic = rightProx.GetRangeInches();
			angleChangle = pid.PIDAngle(currentAngle, desiredAngle); //angle drive value
			if (fabs(angleChangle) < .01) { //if done driving angle
				yOutput = pid.PIDY(leftUltrasonic, rightUltrasonic); //y drive value
			} else {
				yOutput = 0.0; //don't drive y until done driving angle
			}
			if (fabs(angleChangle) < .01 && fabs(yOutput) < .01) { //exit loop
				failsafe = 200;
				isDone = true;
			}
			robotDrive.MecanumDrive_Cartesian(0.0, yOutput, angleChangle); //turn and drive
			frc::Wait(.01); //wait to avoid hogging cpu cycles
			failsafe++;
		}
		robotDrive.MecanumDrive_Cartesian(0.0, 0.0, 0.0); //stop moving
		failsafe = 0; //reset failsafe
		break;
	}

	float bigFailsafe = 0;

	while (!(fabs(aimer.TwoCameraAngleFilter()) <= 3.0) && bigFailsafe < 500 && !IsOperatorControl())//This adjusts the accuracy of the aiming of the robot
	{
		int sign = (aimer.TwoCameraAngleFilter() < 0) ? -1 : 1;//Which direction to turn
		while (aimer.TwoCameraAngleFilter() < 20 * sign && failsafe < 100 && !IsOperatorControl())
		{
			float angleChangle = pid.PIDAngle(gyro.GetYaw(), aimer.TwoCameraAngleFilter() + gyro.GetYaw() + 20 * sign);
			robotDrive.MecanumDrive_Cartesian(0.0, 0.0, angleChangle);
			SmartDashboard::PutNumber("Angle to Gear", aimer.TwoCameraAngleFilter());
			frc::Wait(.01);
			failsafe++;
			bigFailsafe++;
		}
		failsafe = 0;
		while (gyro.GetYaw() + aimer.TwoCameraAngleFilter() >= desiredAngle - 3 && gyro.GetYaw() + aimer.TwoCameraAngleFilter() <= desiredAngle + 3 && failsafe < 200 && !IsOperatorControl())
		{
			float angleChangle = pid.PIDAngle(gyro.GetYaw(), aimer.TwoCameraAngleFilter() + gyro.GetYaw() + 20 * sign);
			robotDrive.MecanumDrive_Cartesian(0, -0.5, angleChangle);
			SmartDashboard::PutNumber("Angle to Gear", aimer.TwoCameraAngleFilter());
			frc::Wait(.01);
			failsafe++;
		}
		failsafe = 0;
		while (!(gyro.GetYaw() >= desiredAngle) && failsafe < 100 && !IsOperatorControl())
		{
			float angleChangle = pid.PIDAngle(gyro.GetYaw(), aimer.TwoCameraAngleFilter() + gyro.GetYaw());
			robotDrive.MecanumDrive_Cartesian(0, 0.0, angleChangle);
			SmartDashboard::PutNumber("Angle to Gear", aimer.TwoCameraAngleFilter());
			frc::Wait(.01);
			failsafe++;
		}
		failsafe = 0;
	}

	while(filter.ultrasonicFilter(leftUltrasonic, rightUltrasonic) > 12 && failsafe < 200 && !IsOperatorControl())
	{
		float angleChangle = pid.PIDAngle(gyro.GetYaw(), aimer.TwoCameraAngleFilter() + gyro.GetYaw());
		float driveSpeed = 0.5 - ((getAverageDistance(leftProx, rightProx) / 80) - 0.5);
		robotDrive.MecanumDrive_Cartesian(0, -driveSpeed, angleChangle);
		SmartDashboard::PutNumber("Angle to Gear", aimer.TwoCameraAngleFilter());
		SmartDashboard::PutNumber("leftProx", leftUltrasonic);
		SmartDashboard::PutNumber("rightProx", rightUltrasonic);
		SmartDashboard::PutNumber("leftIR", leftIR.get());
		SmartDashboard::PutNumber("rightIR", rightIR.get());
		SmartDashboard::PutNumber("Ultrasonic Filter", filter.ultrasonicFilter(leftUltrasonic, rightUltrasonic));
		frc::Wait(.01);
		failsafe++;
	}

	//drop gear

	robotDrive.SetSafetyEnabled(true);
}

inline float getAverageDistance(const Ultrasonic& leftProx, const Ultrasonic& rightProx)
{
	return ((float)leftProx.GetRangeInches() + (float)rightProx.GetRangeInches()) / 2.0;
}

START_ROBOT_CLASS(Robot)
