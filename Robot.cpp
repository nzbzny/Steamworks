#include "Robot.h"
#include "WPILib.h"
#include <opencv2/core/core.hpp> //TODO: do we need this?

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

/*float byteToFloat(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
    float result;
    unsigned char* sResult = (unsigned char*)&result; // (unsigned char*) might be needed.
    sResult[3] = b3;
    sResult[2] = b2;
    sResult[1] = b1;
    sResult[0] = b0;
    return result;
}*/

Robot::Robot() :
		frontLeftMotor(Constants::frontLeftDriveChannel),
		rearLeftMotor(Constants::rearLeftDriveChannel),
		frontRightMotor(Constants::frontRightDriveChannel),
		rearRightMotor(Constants::rearRightDriveChannel),
		robotDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor),
		driveStick(Constants::driveStickChannel),
		operatorStick(Constants::operatorStickChannel),
		gyro(SPI::Port::kMXP, 200),
		pid(),
		aimer(),
		leftProx(Constants::leftProxPinA, Constants::leftProxPinB),
		rightProx(Constants::rightProxPinA, Constants::rightProxPinB),
		leftIR(5),
		rightIR(4),
		gear(Constants::gearActuatorInSole, Constants::gearActuatorOutSole, Constants::gearPusherInSole, Constants::gearPusherOutSole),
		shooter(Constants::rotatorChannel, Constants::shooterChannel, Constants::agitatorChannel),
		compressor(),
		filter(),
		intake(Constants::intakeMotorPin, Constants::verticalConveyorMotorPin),
		climber(Constants::climberPin),
		brakes(Constants::brakesInSole, Constants::brakesOutSole),
		pdp(),
		encoder(NULL), //TODO: make sure we need this
		yEncoder(Constants::yOmniEncoderPinA, Constants::yOmniEncoderPinB, false, Encoder::EncodingType::k4X),
		xEncoder(Constants::xOmniEncoderPinA, Constants::xOmniEncoderPinB, false, Encoder::EncodingType::k4X),
		arduino(I2C::kOnboard, 6)
{
	//yEncoder = new Encoder(Constants::yEncoderPinA, Constants::yEncoderPinB, false, Encoder::EncodingType::k4X);
	//xEncoder = new Encoder(Constants::xEncoderPinA, Constants::xEncoderPinB, false, Encoder::EncodingType::k4X)
	encoder = new Encoder(99,99,false,Encoder::EncodingType::k4X);
	yEncoder.SetDistancePerPulse(.001989);
	xEncoder.SetDistancePerPulse(.001989);
	encoder->SetDistancePerPulse(.001989);
	robotDrive.SetExpiration(0.1);
	gyro.ZeroYaw();
	robotDrive.SetInvertedMotor(RobotDrive::kFrontLeftMotor, false);
	robotDrive.SetInvertedMotor(RobotDrive::kRearLeftMotor, false);
	robotDrive.SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
	robotDrive.SetInvertedMotor(RobotDrive::kRearRightMotor, true);
}

void Robot::RobotInit() {
	camera0 =  CameraServer::GetInstance()->StartAutomaticCapture();
	camera0.SetResolution(320, 240);
	camera0.SetExposureManual(78);
	camera0.SetFPS(24);
//	camera1 = CameraServer::GetInstance()->StartAutomaticCapture(1);
//	camera1.SetResolution(320, 240);
//	camera1.SetExposureManual(312);
	NetworkTable::SetUpdateRate(.01);
	SmartDashboard::PutNumber("Joe P-value", -(Constants::accumulatorPower));
	SmartDashboard::PutNumber("shooterSpeed", .9);
	SmartDashboard::PutNumber("Starting Position", 3);
	SmartDashboard::PutNumber("Joe x offset", -1);
	pid.setAngle(SmartDashboard::GetNumber("angle_p", Constants::angle_p_default), SmartDashboard::GetNumber("angle_i", Constants::angle_i_default), SmartDashboard::GetNumber("angle_d", Constants::angle_d_default));
	pid.setY(SmartDashboard::GetNumber("y_p", Constants::y_p_default), SmartDashboard::GetNumber("y_i", Constants::y_i_default), SmartDashboard::GetNumber("y_d", Constants::y_d_default));
	pid.setX(SmartDashboard::GetNumber("x_p", Constants::x_p_default), SmartDashboard::GetNumber("x_i", Constants::x_i_default), SmartDashboard::GetNumber("x_d", Constants::x_d_default));
	gyro.Reset();
	gyro.ResetDisplacement();
	leftProx.SetAutomaticMode(true);
	rightProx.SetAutomaticMode(true);
}

/**
 * Runs the motors with Mecanum drive.
 */

inline float getAverageDistance(const Ultrasonic& leftProx, const Ultrasonic& rightProx);

void Robot::OperatorControl()
{
	robotDrive.SetSafetyEnabled(false);
	pid.setAngle(SmartDashboard::GetNumber("angle_p", Constants::angle_p_default), SmartDashboard::GetNumber("angle_i", Constants::angle_i_default), SmartDashboard::GetNumber("angle_d", Constants::angle_d_default));
	pid.setY(SmartDashboard::GetNumber("y_p", Constants::y_p_default), SmartDashboard::GetNumber("y_i", Constants::y_i_default), SmartDashboard::GetNumber("y_d", Constants::y_d_default));
	pid.setX(SmartDashboard::GetNumber("x_p", Constants::x_p_default), SmartDashboard::GetNumber("x_i", Constants::x_i_default), SmartDashboard::GetNumber("x_d", Constants::x_d_default));
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

	//auto move
	bool inAutoMoveLoop = false; //to force quit field oriented drive when in an auto move loop - auto move only works in robot oriented drive

	//shooter
	bool shooterAutoAngleButtonPressed = false;
	bool shooterShootButtonPressed = false;
	bool shooting = false;
	float shooterSpeed = 0.0;
	bool shooterAngleReached = true;
	Timer reverseAgitatorTimer;
	bool reverseAgitating = false;
	bool reverseAgitatorButtonPressed = false;
	shooterSpeed = SmartDashboard::GetNumber("shooterSpeed", .9);
	//bool shooterAgitateButtonPressed = false;
	//bool agitating = false;
	float absoluteBoilerAngle = 0.0;

	//gear
	bool gearButtonPressed = false;
	bool gearPusherButtonPressed = false;
	bool gearDropButtonPressed = false;
	bool autoGearUp = false;
	Timer gearOpenTimer;
	gearOpenTimer.Start();

	//one axis
	bool oneAxisButtonPressed = false;
	float oneAxisDesiredAngle = 0.0;

	//ultrasonics
	float leftUltrasonic = leftProx.GetRangeInches();
	float rightUltrasonic = rightProx.GetRangeInches();

	//field oriented driveZAxis
	bool fieldOrientedDrive = false;
	bool fieldOrientedDriveButtonPressed = false;

	//intake
	bool intakeButtonPressed = false;
	bool intakeRunning = true;
	float intakeCurrent = 0.0;

	//climber
	bool climberButtonPressed = false;
	bool climbing = false;
	bool climberMinPowerButtonPressed = false;
	bool climbingMinPower = false;

	//brakes
	//bool brakeButtonPressed = false;

	//flip drive orientation
	int flipDriveOrientation = 1;

	//full power drive
	bool driveFullPower = false;
	bool driveFullPowerButtonPressed = false;

	//init functions
	//filter.initializeLastUltrasonics(leftUltrasonic, rightUltrasonic);
	//filter.initializePredictedValue(leftUltrasonic, rightUltrasonic);
	leftProx.SetAutomaticMode(true);
	rightProx.SetAutomaticMode(true);
	shooter.enable();
	climber.enable();
	compressor.Start();

	//accumulator
	Accumulator accum((float)0.0, (float)0.0, (float)0.0, 2, leftProx, rightProx, aimer, *encoder, *encoder, pid);
	// NOAH *******  check the encoder we are passing is valid

	double lastLeftProxValue = 0;
	double lastLeftProxTime = 0;
	double lastRightProxValue = 0;
	double lastrightProxTime = 0;
	double lastEncoderDistance = 0;
	DoubleDouble driveVals(0, 0, 0);
	double yTargetDistance = -9999.0;
	bool inYTargetMode = false;
	float joeDriveX = 0.0;
	float joeDriveY = 0.0;
	float joeDriveZ = 0.0;

	//camera switching
	int cameraInOperation = 1;
	bool cameraSwitching = false;
	bool cameraSwapButtonPressed = false;


	float targetAngle = 0;

	//new ultrasonics
	/*float ultrasonicFrontRight = 0;
	float ultrasonicFrontLeft = 0;
	float ultrasonicCenterRight = 0;
	float ultrasonicCenterLeft = 0;
	float ultrasonicBackRight = 0;
	float ultrasonicBackLeft = 0;
	uint8_t toSend[10];//array of bytes to send over I2C
	uint8_t toReceive[50];//array of bytes to receive over I2C
	uint8_t numToSend = 2;//number of bytes to send
	uint8_t numToReceive = 28;//number of bytes to receive
	uint8_t lightcannonFront = 0;
	uint8_t lightcannonBack = 0;
	SmartDashboard::PutNumber("ultraFR", ultrasonicFrontRight);
	SmartDashboard::PutNumber("ultraFL", ultrasonicFrontLeft);
	SmartDashboard::PutNumber("ultraCR", ultrasonicCenterRight);
	SmartDashboard::PutNumber("ultraCL", ultrasonicCenterLeft);
	SmartDashboard::PutNumber("ultraBR", ultrasonicBackRight);
	SmartDashboard::PutNumber("ultraBL", ultrasonicBackLeft);*/

	//initial checks
	gear.setBottom(false);
	gear.setPusher(false);

	while (IsOperatorControl() && IsEnabled())
	{

//		if (operatorStick.GetRawButton(7)) {
//			robotDrive.MecanumDrive_Cartesian(0.0, 1.0, 0.0);
//			Wait(1);
//			robotDrive.MecanumDrive_Cartesian(0.0, 0.0, 0.0);
//		}


		/*
		 *
		 * BASIC CHECKS
		 *
		 */

		 gyroValid = gyro.IsConnected();
		 SmartDashboard::PutBoolean("gyro alive", gyroValid);
		 calibrating = gyro.IsCalibrating();
		 voltage = DriverStation::GetInstance().GetBatteryVoltage();
		 angleOutput = 0; //reset output so that if the pid loop isn't being called it's not reserved from the last time it's called
		 angle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw(); //TODO: swap front back
		 yOutput = 0; //reset output
		 xOutput = 0;
		 leftUltrasonic = leftProx.GetRangeInches();
		 rightUltrasonic = rightProx.GetRangeInches();
		 /*if (driveStick.GetRawButton(Constants::cancelAllButton)) {
			 shooterAutoAngleButtonPressed = false;
			 shooterShootButtonPressed = false;
			 shooting = false;
			 shooterSpeed = 0.0;
			 shooterAngleReached = true;
			 intakeButtonPressed = false;
			 intakeRunning = false;
			 climberButtonPressed = false;
			 climbing = false;
			 gearButtonPressed = true;
			 gearOpenCounter = 1001;
			 gear.setBottom(false); //bottom closed - TODO: may need to flip
			 shooter.stop();
			 climber.setSpeed(0.0);
			 intake.runIntake(0.0);
			 intake.runVerticalConveyor(0.0);
		 }*/ //TODO: uncomment

		if(operatorStick.GetRawButton(12)){
			gyro.ZeroYaw();
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
			shooterAngleReached = shooter.setAngle(aimer.GetAngleToShoot()); //angle from tj's vision code
			shooterAutoAngleButtonPressed = true; //set button pressed to true so that holding the button for more than 10ms (loop time) doesn't activate the loop above and cancel it
		}
		if (!driveStick.GetRawButton(Constants::shooterAutoAngleButton)) { //if the shooter button has been let go
			shooterAutoAngleButtonPressed = false; //set to false so it can be pressed again
		}
		if (!shooterAngleReached) { //if the shooter angle hasn't yet been reached
			shooterAngleReached = shooter.setAngle(aimer.GetAngleToShoot()); //still shooting
		}
		if (driveStick.GetRawButton(Constants::shooterShootButton) && !shooterShootButtonPressed && !shooting) { //shoot
			//TODO: get shooter speed
			shooter.shoot(shooterSpeed); //shoot at the speed
			shooterShootButtonPressed = true; //button is pressed so it doesn't immediately cancel
			shooting = true; //set shooting to true so it knows next time the button is pressed to cancel
			//agitating = true;
		} else if (driveStick.GetRawButton(Constants::shooterShootButton) && !shooterShootButtonPressed && shooting) { //cancel
			shooter.setSpeed(0.0); //turn off shooter
			shooter.agitate(0.0);
			reverseAgitating = false;
			shooterShootButtonPressed = true; //so it doesn't immediately start shooting again
			shooting = false; //so it knows what loop to go into
			//agitating = false;
		}
		if (driveStick.GetRawButton(Constants::reverseAgitatorButton) && !reverseAgitatorButtonPressed && shooting) { //clean up the agitator when it gets jammed
			reverseAgitating = !reverseAgitating; //set bool to avoid flipping it right back while shooting
			reverseAgitatorTimer.Reset(); //reset the timer
			reverseAgitatorButtonPressed = true;
			shooter.agitate(-1.0); //reverse the agitator direction
		} else if (!driveStick.GetRawButton(Constants::reverseAgitatorButton)) {
			reverseAgitatorButtonPressed = false;
		}
		if (shooting && !reverseAgitating) { //if shooting and agitator going the normal way
			shooter.shoot(shooterSpeed); //shoot and agitate
		}
		if (reverseAgitating) { //if the agitator should be reverse
			shooter.agitate(-1.0); //usually it's set to --1.0
		}
		if (reverseAgitatorTimer.Get() > 2) { //if it's been reversed for > 2 seconds
			reverseAgitating = false; //stop reversing the agitator
		}
		if (!driveStick.GetRawButton(Constants::shooterShootButton)) { //when button is let go
			shooterShootButtonPressed = false; //let shooter change state
		}

		if (fabs(operatorStick.GetRawAxis(1)) > .05) {
			shooter.move(scaleJoysticks(operatorStick.GetRawAxis(1), Constants::driveYDeadZone, Constants::driveYMax, Constants::driveYDegree)); //TODO: delete after testing
		}

		if (driveStick.GetRawButton(Constants::shooterAutoAngleButton)) { //auto turn to the target
			absoluteBoilerAngle = fmod(angle + aimer.GetBoilerAngle(), 360);
			absoluteBoilerAngle = absoluteBoilerAngle < 0 ? absoluteBoilerAngle + 360 : absoluteBoilerAngle;
			angleOutput = pid.PIDAngle(angle, absoluteBoilerAngle); //turn to boiler
		}
		/*if (driveStick.GetRawButton(Constants::agitateButton) && !shooterAgitateButtonPressed) { //TODO: make toggle
			agitating = !agitating;
			shooterAgitateButtonPressed = true;
		} else if (!driveStick.GetRawButton(Constants::agitateButton)) {
			shooterAgitateButtonPressed = false;
		}
		if (agitating) {
			shooter.agitate(-1.0);
		} else {
			shooter.agitate(0.0);
		}*/

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

		if(driveStick.GetRawButton(Constants::gearActuateButton) && !gearButtonPressed) { //open / close gear
			if (!gear.getBottom()) { //if closed
				gear.setBottom(true); //set to the opposite of what it currently is
				Wait(.7);
				gear.setPusher(true);
				autoGearUp = true;
			} else { //if open
				gear.setPusher(false);
				Wait(.5);
				gear.setBottom(false);
				autoGearUp = false;
			}

			gearButtonPressed = true; //button still pressed = true
		} else if (!driveStick.GetRawButton(Constants::gearActuateButton)) { //button not pressed - so it doesn't keep flipping between open and closed
			gearButtonPressed = false; //button not pressed
		}
		if (!gear.getBottom() || !autoGearUp) { //TODO: may need to make a not
			gearOpenTimer.Reset(); //reset timer to zero so it doesn't open after 5 seconds
		}
		if (gearOpenTimer.Get() > 3) {
			gear.setPusher(false);
		}
		if (gearOpenTimer.Get() > 10) { //if it's been open for more than 10 seconds
			gear.setBottom(!gear.getBottom()); //set to closed
			gearOpenTimer.Reset();
			autoGearUp = false;
		}

		if (driveStick.GetRawButton(Constants::gearPusherButton) && !gearPusherButtonPressed && gear.getBottom()) {
			gear.setPusher(!gear.getPusher());
			gearPusherButtonPressed = true;
		} else if (!driveStick.GetRawButton(Constants::gearPusherButton)) {
			gearPusherButtonPressed = false;
		}

		if (driveStick.GetRawButton(Constants::gearDropButton) && !gearDropButtonPressed) {
			gear.setBottom(!gear.getBottom());
			gearDropButtonPressed = true;
		} else if (!driveStick.GetRawButton(Constants::gearDropButton)) {
			gearDropButtonPressed = false;
		}

		//gear.get = true - open
		//gear.get = false - closed



		/*
		 *
		 * END GEAR CODE
		 *
		 */

		/*
		 *
		 * INTAKE CODE
		 *
		 */

		intakeCurrent = pdp.GetCurrent(Constants::intakePDPChannel);
		if (driveStick.GetRawButton(Constants::intakeActivateButton) && !intakeButtonPressed) {
			intakeRunning = !intakeRunning; //if on, turn off. If off, turn on
			intakeButtonPressed = true; //doesn't keep flipping when held down
		} else if (!driveStick.GetRawButton(Constants::intakeActivateButton)) {
			intakeButtonPressed = false; //lets you flip the state again
		}
		if (intakeRunning) { //if it's turned on
			intake.runIntake(Constants::intakeRunSpeed); //run intake
			intake.runVerticalConveyor(Constants::verticalConveyorRunSpeed); //run vertical conveyor
		} else { //if it's turned off
			intake.runIntake(0.0); //stop intake
			intake.runVerticalConveyor(0.0); //stop vertical conveyor
		}
		if (intakeCurrent > Constants::intakeCurrentMax) { //if the current is too high, turn off - SAFETY
			intake.runIntake(0.0); //off
			intake.runVerticalConveyor(0.0); //off
		}

		/*
		 *
		 * END INTAKE CODE
		 *
		 */



		/*
		 *
		 * CLIMBER CODE
		 *
		 */

		/*if (driveStick.GetRawButton(Constants::climbButton) && !climberButtonPressed) {
			climbing = !climbing; //flip state
			climberButtonPressed = true; //don't keep flipping states when held down
		} else if (!driveStick.GetRawButton(Constants::climbButton)) { //if not pressed anymore
			climberButtonPressed = false; //lets you flip state again
		}
		if (climbing) { //if it's running
			climber.setSpeed(Constants::climberRunSpeed); //run
		} else { //if it's not running
			climber.setSpeed(0.0); //stop
		}*/

		if (driveStick.GetRawButton(Constants::climbButton) && !climberButtonPressed) {
			climbing = !climbing;
			climbingMinPower = false;
			climberButtonPressed = true;
		} else if (!driveStick.GetRawButton(Constants::climbButton)) {
			climberButtonPressed = false;
		}
		if (driveStick.GetRawButton(Constants::climberMinPowerButton) && !climberMinPowerButtonPressed) {
			climbingMinPower = !climbingMinPower;
			climberMinPowerButtonPressed = true;
			intakeRunning = false;
		} else if (driveStick.GetRawButton(Constants::climberMinPowerButton)) {
			climberMinPowerButtonPressed = false;
		}
		if (climbingMinPower) {
			climbing = false;
			climber.setSpeed(-.15); //climb min power
		}

		if (climbing) {
			climber.setSpeed(-1.0);
		} else if (driveStick.GetRawButton(Constants::climbDownButton)) {
			climber.setSpeed(1.0);
		} else if (!climbingMinPower && !climbing) {
			climber.setSpeed(0.0);
		}

		if (operatorStick.GetRawButton(Constants::climberSlowMoOperatorButton)) {
			climber.setSpeed(.25);
		}

		if (operatorStick.GetRawButton(Constants::climberSlowMoBackwardsOperatorButton)) {
			climber.setSpeed(-.25);
		}

		/*
		 *
		 * END CLIMBER CODE
		 */

		/*
		 *
		 * BRAKE CODE
		 *
		 */

		/*if (driveStick.GetRawButton(Constants::brakeButton) && !brakeButtonPressed) { //if activating brakes
			brakes.set(!brakes.get()); //flip state
			brakeButtonPressed = true; //don't flip state when held down
		} else if (!driveStick.GetRawButton(Constants::brakeButton)) { //if not pressed anymore
			brakeButtonPressed = false; //lets you flip state again
		}*/ //BRAKES ARE OFF AT THE MOMENT

		/*
		 *
		 * END BRAKE CODE
		 *
		 */

		/*
		 *
		 * FLIP DRIVE ORIENTATION CODE
		 *
		 */

		if (driveStick.GetRawButton(Constants::swapDriveButton)) {
			flipDriveOrientation = -1;
		} else {
			flipDriveOrientation = 1;
		}

		/*
		 *
		 * END FLIP DRIVE ORIENTATION CODE
		 *
		 */

		/*
		 *
		 * PID CODE
		 *
		 */

		if(driveStick.GetPOV() != -1 && gyroValid) { //turn to angle 0, 90, 180, 270
			angleOutput = pid.PIDAngle(angle, driveStick.GetPOV()); //call pid loop
			inAutoMoveLoop = true; //auto moving - force quit field oriented drive
		} else {
			pid.resetPIDAngle(); //if loop is done reset values
			inAutoMoveLoop = false; //reopen field oriented drive (if active)
		}

		/*if(driveStick.GetRawButton(Constants::moveToGearButton)) { //pid move
			angleOutput = pid.PIDAngle(angle, 0); //TODO: get angle via function (closest angle of the 3 options (0, 60, -60, find which is closest to current angle)?)
			if (angleOutput == 0) {
				xOutput = filter.ultrasonicFilter(leftUltrasonic, rightUltrasonic) > 45 ? pid.PIDX(aimer.TwoCameraAngleFilter()) : 0.0; //if done turning move x
				yOutput = (xOutput < .01) ? pid.PIDY(leftUltrasonic, rightUltrasonic) : 0.0; //if done moving x move y
			}
			inAutoMoveLoop = true; //force quit field oriented drive
		} else { //if loop is done reset values
			pid.resetPIDX();
			pid.resetPIDY();
			inAutoMoveLoop = false; //reopen field oriented drive
		}*/

		/*
		 *
		 * END PID CODE
		 *
		 */

		/*
		 *
		 * JOSEPH MOVE TEST CODE
		 *
		 */

		if(driveStick.GetRawButton(2))
		{
			if (!inYTargetMode)
			{
				if (angle < 30 || angle > 330)
					targetAngle = 0;
				else if (angle >= 30 && angle < 180)
					targetAngle = 60;
				else
					targetAngle = 300;
			}
			driveVals = accum.drive(inYTargetMode, true, false, false, 4, targetAngle, 2, angle);

			joeDriveX = driveVals.x;
			joeDriveY = driveVals.y;
			joeDriveZ = driveVals.angle;
			inYTargetMode = true;
			SmartDashboard::PutNumber("Joe target Angle", 0);
			SmartDashboard::PutBoolean("Joe in loop", true);
			SmartDashboard::PutNumber("Joe DriveX", joeDriveX);
			SmartDashboard::PutNumber("Joe DriveY", joeDriveY);
			SmartDashboard::PutNumber("Joe DriveZ", joeDriveZ);
			SmartDashboard::PutNumber("Button Pushed", 2.0);
			robotDrive.MecanumDrive_Cartesian(joeDriveX, joeDriveY, joeDriveZ);
		}
		else
		{
			SmartDashboard::PutBoolean("Joe in loop", false);
			inYTargetMode = false;
		}



		SmartDashboard::PutNumber("Joseph driveX", driveX);
		SmartDashboard::PutNumber("Joseph driveY", driveY);
		SmartDashboard::PutNumber("Joseph driveZ", driveZ);
		//robotDrive.MecanumDrive_Cartesian(driveX, driveY, driveZ); //drive

		/*
		 *
		 * END JOSEPH MOVE TEST CODE
		 *
		 */

		/*
		 *
		 * DRIVE CODE
		 *
		 */

		//scaling for the joystick deadzones - TODO: fix scaling for the ps4 controllers
		if (!inYTargetMode) {
			driveX = fabs(driveStick.GetRawAxis(Constants::driveXAxis)) > .1 ? driveStick.GetRawAxis(Constants::driveXAxis) : 0.0;
			driveY = fabs(driveStick.GetRawAxis(Constants::driveYAxis)) > .05 ? driveStick.GetRawAxis(Constants::driveYAxis) : 0.0;
			driveZ = fabs(driveStick.GetRawAxis(Constants::driveZAxis)) > .05 ? driveStick.GetRawAxis(Constants::driveZAxis) : 0.0;

			if (driveStick.GetRawButton(Constants::driveOneAxisButton)) { //drive only one axis
				if (!oneAxisButtonPressed) { //if not in the middle of being held down
					oneAxisButtonPressed = true; //being held down now - don't reset desired angle
					oneAxisDesiredAngle = angle; //set desired angle
				}
				if (fabs(driveX) > fabs(driveY) && fabs(driveX) > fabs(driveZ)) { //if X is greater than Y and Z, then it will only go in the direction of X
					angleOutput = pid.PIDAngle(angle, oneAxisDesiredAngle); //stay straight //TODO: uncomment
					driveY = 0;
					driveZ = 0;
				}
				else if (fabs(driveY) > fabs(driveX) && fabs(driveY) > fabs(driveZ)) { //if Y is greater than X and Z, then it will only go in the direction of Y
					angleOutput = pid.PIDAngle(angle, oneAxisDesiredAngle); //stay straight //TODO: uncomment
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

			if (!driveFullPower) {
							driveX *= .5;
							driveY *= .5;
							driveZ *= .5;
			}

			driveX = fabs(driveX + xOutput) > 1 ? std::copysign(1, driveX + xOutput) : driveX + xOutput; //if driving and pid'ing (pls dont) maxes you out at 1 so the motors move
			driveY = fabs(driveY + yOutput) > 1 ? std::copysign(1, driveY + yOutput) : driveY + yOutput; //if driving and pid'ing (pls dont) maxes you out at 1 so the motors move
			driveZ = fabs(driveZ + angleOutput) > 1 ? std::copysign(1, driveZ + angleOutput) : driveZ + angleOutput; //if driving and pid'ing (pls dont) maxes you out at 1 so the motors move

			driveX *= flipDriveOrientation; //flip front / back rotation
			driveY *= flipDriveOrientation;

			if (driveStick.GetRawButton(Constants::driveFullPowerToggleButton) && !driveFullPowerButtonPressed) {
				driveFullPower = !driveFullPower;
				driveFullPowerButtonPressed = true;
			} else if (!driveStick.GetRawButton(Constants::driveFullPowerToggleButton)) {
				driveFullPowerButtonPressed = false;
			}

			if (driveStick.GetRawButton(Constants::fieldOrientedDriveButton) && !fieldOrientedDriveButtonPressed) {
				fieldOrientedDrive = !fieldOrientedDrive;
				fieldOrientedDriveButtonPressed = true;
			} else if (!driveStick.GetRawButton(Constants::fieldOrientedDriveButton)) {
				fieldOrientedDriveButtonPressed = false;
			}
			if (fieldOrientedDrive && !inYTargetMode) {
				robotDrive.MecanumDrive_Cartesian(driveX, driveY, driveZ, angle); //field oriented drive
			} else {
				robotDrive.MecanumDrive_Cartesian(driveX, driveY, driveZ); //robot oriented drive
			}

		}

			/*
			 *
			 * DRIVE DIRECTIONS
			 *
			 */
				//positive x is gear left and intake right
				//positive y is towards the gear
				//positive z is clockwise
			/*
			 *
			 * END DRIVE DIRECTIONS
			 *
			 */

		/*
		 *
		 * END DRIVE CODE
		 *
		 */

		/*
		 *
		 * CAMERA SWAP CODE
		 *
		 */


		if (driveStick.GetRawButton(9) && !cameraSwapButtonPressed) {
			cameraInOperation++;
			cameraInOperation = cameraInOperation % 2;
			SmartDashboard::PutNumber("camera in operation", cameraInOperation);
			if (cameraInOperation == 0) {
				CameraServer::GetInstance()->PutVideo(camera0.GetName(), 640, 480);
			} else {
				CameraServer::GetInstance()->PutVideo(camera1.GetName(), 640, 480);

			}
			cameraSwapButtonPressed = true;
		} else if (!driveStick.GetRawButton(9)) {
			cameraSwapButtonPressed = false;
		}

		/*
		 *
		 * END CAMERA SWAP CODE
		 *
		 */

		/*
		 *
		 * NEW ULTRASONIC CODE (ARDUINO)
		 *
		 */

		/*lightcannonFront = 20;
		lightcannonBack = 20;
		toSend[0] = lightcannonFront;
		toSend[1] = lightcannonBack;
		arduino.Transaction(toSend, numToSend, toReceive, numToReceive);
		ultrasonicFrontLeft = byteToFloat(toReceive[4], toReceive[5], toReceive[6], toReceive[7]);
		ultrasonicFrontRight = byteToFloat(toReceive[8], toReceive[9], toReceive[10], toReceive[11]);
		ultrasonicCenterRight = byteToFloat(toReceive[12], toReceive[13], toReceive[14], toReceive[15]);
		ultrasonicCenterLeft = byteToFloat(toReceive[16], toReceive[17], toReceive[18], toReceive[19]);
		ultrasonicBackRight = byteToFloat(toReceive[20], toReceive[21], toReceive[22], toReceive[23]);
		ultrasonicBackLeft = byteToFloat(toReceive[24], toReceive[25], toReceive[26], toReceive[27]);
		SmartDashboard::PutNumber("ultraFR", ultrasonicFrontRight);
		SmartDashboard::PutNumber("ultraFL", ultrasonicFrontLeft);
		SmartDashboard::PutNumber("ultraCR", ultrasonicCenterRight);
		SmartDashboard::PutNumber("ultraCL", ultrasonicCenterLeft);
		SmartDashboard::PutNumber("ultraBR", ultrasonicBackRight);
		SmartDashboard::PutNumber("ultraBL", ultrasonicBackLeft);*/


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
		SmartDashboard::PutNumber("Encoder value", encoder->GetDistance());
//		SmartDashboard::PutNumberArray("leftAngleArray", aimer.GetLeftAngleArray());
//		SmartDashboard::PutNumberArray("rightAngleArray", aimer.GetRightAngleArray());
		SmartDashboard::PutBoolean("calibrateButtonPushed", calibrating);
		SmartDashboard::PutNumber("voltage", voltage);
		//SmartDashboard::PutNumber("Ultrasonic Filter", filter.ultrasonicFilter(leftUltrasonic, rightUltrasonic));
		//SmartDashboard::PutNumber("AngleToGear", aimer.GetAngleToGear());
		SmartDashboard::PutBoolean("IsCalibrating", gyro.IsCalibrating()); //TODO: start of issue
//		SmartDashboard::PutNumber("Ultrasonic Kalman Filter", filter.kalmanFilter(leftUltrasonic, rightUltrasonic, driveY));
//		SmartDashboard::PutBoolean("Braking", brakes.get());
		SmartDashboard::PutBoolean("Gear Open", gear.getBottom()); //TODO: end of issue
		SmartDashboard::PutNumber("Intake Current", intakeCurrent);
		SmartDashboard::PutNumber("shooter pitch", shooter.Pitch());
		SmartDashboard::PutNumber("shooter roll", shooter.Roll());
		SmartDashboard::PutNumber("operator y axis", operatorStick.GetRawAxis(1));
		SmartDashboard::PutBoolean("shooter shoot button pressed", shooterShootButtonPressed);
		SmartDashboard::PutBoolean("shooting", shooting);
		SmartDashboard::PutBoolean("Compy Switch", compressor.GetPressureSwitchValue());
		SmartDashboard::PutNumber("gear open timer", gearOpenTimer.Get());
		SmartDashboard::PutNumber("shooterSpeed", shooterSpeed);
		SmartDashboard::PutNumber("shooterEnc", shooter.getEncoder());
		SmartDashboard::PutNumber("yEncoderTicks", yEncoder.Get());
		SmartDashboard::PutNumber("xEncoderTicks", xEncoder.Get());
		SmartDashboard::PutNumber("yEncoderDistance", yEncoder.GetDistance());
		SmartDashboard::PutNumber("xEncoderDistance", xEncoder.GetDistance());
		SmartDashboard::PutNumber("shooterRPM", shooterSpeed * 4196);
		SmartDashboard::PutBoolean("FULL POWER DRIVE", driveFullPower);
		SmartDashboard::PutNumber("driveX", driveX);
		SmartDashboard::PutNumber("driveY", driveY);
		SmartDashboard::PutNumber("driveZ", driveZ);
		SmartDashboard::PutNumber("climber current", pdp.GetCurrent(Constants::climberPin));
		SmartDashboard::PutNumber("climber current graph", pdp.GetCurrent(Constants::climberPin));
		SmartDashboard::PutNumber("front left bus voltage", frontLeftMotor.GetBusVoltage());
		//SmartDashboard::PutNumber("angle_p", .02);
		//SmartDashboard::PutNumber("angle_i", .001);
		//SmartDashboard::PutNumber("angle_d", .001);

		/*
		 *
		 * END SMART DASHBOARD
		 *
		 */

		frc::Wait(0.01); // wait 5ms to avoid hogging CPU cycles TODO: change back to .005 if necessary

	}
	robotDrive.SetSafetyEnabled(true);
	shooter.disable();
	climber.disable();
	compressor.Stop();
}

//void Robot::Autonomous() {
//	//This function so far will go through and try to target onto the peg and try to run onto it.
////	robotDrive.SetSafetyEnabled(false);
////	gyro.ResetDisplacement();
////	gyro.ZeroYaw();
////	gear.setBottom(true); //TODO: may need to change to false
////	xEncoder.Reset();
////	yEncoder.Reset();
////	Accumulator accum((float)0.0, (float)0.0, (float)0.0, 2, leftProx, rightProx, aimer, *encoder, *encoder, pid);
////	int failsafe = 0;
////	float desiredAngle = 0.0;//This is the angle to which the robot will try to aim
////	float angleChangle = 0.0;
////	float yOutput = 0.0;
////	float currentAngle = 0.0;
////	float leftUltrasonic = leftProx.GetRangeInches();
////	float rightUltrasonic = rightProx.GetRangeInches();
////	bool isDone, started;
////	float driveX = 0.001, driveY = 0.001, driveZ = 0.001;
////	switch((int)SmartDashboard::GetNumber("Starting Position", 1))//This gets the starting position from the user
////	{
////	case 1://Position 1: straight from the middle peg
////		//I think we should just be able to go straight from here, dude
////		desiredAngle = 0.0;
////		isDone = false;
////		started = false;
////		while (driveX != 0 && driveY != 0 && driveZ != 0 && failsafe < 2000 && !IsOperatorControl() && IsEnabled()) {
////			DoubleDouble driveVals = accum.drive(started, true, false, false, 8, desiredAngle, 2, gyro.GetYaw());
////			started = true;
////			driveX = driveVals.x;
////			driveY = driveVals.y;
////			driveZ = driveVals.angle;
////			robotDrive.MecanumDrive_Cartesian(driveX, driveY, driveZ); //drive forward while staying straight
////			failsafe++;
////			/*if (filter.ultrasonicFilter(leftUltrasonic, rightUltrasonic) < 12) { //TODO: change when we get the kalman filters on
////				isDone = true; //stop the loop
////				failsafe = 400;
////			}*/
////			/*if (driveY == 0 && driveX == 0)
////				isDone = true;
////			frc::Wait(.01); //wait to avoid hogging cpu cycles
////			failsafe++; //increment failsafe variable
////		}
////		robotDrive.MecanumDrive_Cartesian(0.0, 0.0, 0.0); //stop robot
////		failsafe = 0; //reset failsafe for use later*/
////		}
////		gear.setBottom(false);
////		frc::Wait(.5);
////		robotDrive.MecanumDrive_Cartesian(0, -Constants::minForwardPower, 0);
////		frc::Wait(3);
////		robotDrive.MecanumDrive_Cartesian(0, 0, 0);
////		break;
////	case 2://Position 2: on the left
////		desiredAngle = 60;
////		isDone = false;
////		started = false;
////		while(yEncoder.GetDistance() < 90)
////			robotDrive.MecanumDrive_Cartesian(0, 1.0, pid.PIDAngle(gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw(), desiredAngle));
////		while(fabs(gyro.GetYaw() - desiredAngle) > 2)
////			robotDrive.MecanumDrive_Cartesian(0, 0, pid.PIDAngle(gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw(), desiredAngle));
////		frc::Wait(.25);
////		while (driveX != 0 && driveY != 0 && driveZ != 0 && failsafe < 2000 && !IsOperatorControl() && IsEnabled()) {
////			DoubleDouble driveVals = accum.drive(started, true, false, false, 8, desiredAngle, 2, gyro.GetYaw());
////			started = true;
////			driveX = driveVals.x;
////			driveY = driveVals.y;
////			driveZ = driveVals.angle;
////			robotDrive.MecanumDrive_Cartesian(driveX, driveY, driveZ); //drive forward while staying straight
////			failsafe++;
////					/*if (filter.ultrasonicFilter(leftUltrasonic, rightUltrasonic) < 12) { //TODO: change when we get the kalman filters on
////						isDone = true; //stop the loop
////						failsafe = 400;
////					}*/
////					/*if (driveY == 0 && driveX == 0)
////						isDone = true;
////					frc::Wait(.01); //wait to avoid hogging cpu cycles
////					failsafe++; //increment failsafe variable
////				}
////				robotDrive.MecanumDrive_Cartesian(0.0, 0.0, 0.0); //stop robot
////				failsafe = 0; //reset failsafe for use later*/
////		}
////		gear.setBottom(false);
////		frc::Wait(.5);
////		robotDrive.MecanumDrive_Cartesian(0, -Constants::minForwardPower, 0);
////		frc::Wait(3);
////		robotDrive.MecanumDrive_Cartesian(0, 0, 0);
////		break;
////	case 3://Position 2: on the left
////		desiredAngle = 300;
////				isDone = false;
////				started = false;
////				while(yEncoder.GetDistance() < 90)
////					robotDrive.MecanumDrive_Cartesian(0, 1.0, pid.PIDAngle(gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw(), desiredAngle));
////				while(fabs(gyro.GetYaw() - desiredAngle) > 2)
////					robotDrive.MecanumDrive_Cartesian(0, 0, pid.PIDAngle(gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw(), desiredAngle));
////				frc::Wait(.25);
////				while (driveX != 0 && driveY != 0 && driveZ != 0 && failsafe < 2000 && !IsOperatorControl() && IsEnabled()) {
////					DoubleDouble driveVals = accum.drive(started, true, false, false, 8, desiredAngle, 2, gyro.GetYaw());
////					started = true;
////					driveX = driveVals.x;
////					driveY = driveVals.y;
////					driveZ = driveVals.angle;
////					robotDrive.MecanumDrive_Cartesian(driveX, driveY, driveZ); //drive forward while staying straight
////					failsafe++;
////							/*if (filter.ultrasonicFilter(leftUltrasonic, rightUltrasonic) < 12) { //TODO: change when we get the kalman filters on
////								isDone = true; //stop the loop
////								failsafe = 400;
////							}*/
////							/*if (driveY == 0 && driveX == 0)
////								isDone = true;
////							frc::Wait(.01); //wait to avoid hogging cpu cycles
////							failsafe++; //increment failsafe variable
////						}
////						robotDrive.MecanumDrive_Cartesian(0.0, 0.0, 0.0); //stop robot
////						failsafe = 0; //reset failsafe for use later*/
////				}
////				gear.setBottom(false);
////				frc::Wait(.5);
////				robotDrive.MecanumDrive_Cartesian(0, -Constants::minForwardPower, 0);
////				frc::Wait(3);
////				robotDrive.MecanumDrive_Cartesian(0, 0, 0);
////				break;
////	}
//
//
//
//
//
////	float bigFailsafe = 0;
////	failsafe = 0;
////	DoubleDouble driveVals(0, 0, 0);
////	while(failsafe < 500 && driveVals.x > 0.5 && !IsOperatorControl() && IsEnabled())
////	{
////		driveVals = accum.drive(true, true, false, false, 12, 0, 2, gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw());
////		robotDrive.MecanumDrive_Cartesian(driveVals.x, driveVals.y, driveVals.angle);
////		failsafe++;
////	}
////	/*while (!(fabs(aimer.TwoCameraAngleFilter()) <= 3.0) && bigFailsafe < 500 && !IsOperatorControl())//This adjusts the accuracy of the aiming of the robot TODO: change to whatever we're actually using for the camera
////	{
////		int sign = (aimer.TwoCameraAngleFilter() < 0) ? -1 : 1;//Which direction to turn
////		while (fabs(currentAngle - desiredAngle) < 3 && failsafe < 100 && !IsOperatorControl())
////		{
////			currentAngle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw(); //current angle
////			desiredAngle = aimer.TwoCameraAngleFilter() + currentAngle;
////			desiredAngle = desiredAngle < 0 ? desiredAngle + 360 : desiredAngle > 360 ? desiredAngle - 360 : desiredAngle; //nested question mark operator - bada bop bop ba, I'm lovin' it - TODO: change to whatever we're actually using for the camera
////			float angleChangle = pid.PIDAngle(currentAngle, desiredAngle);
////			robotDrive.MecanumDrive_Cartesian(0.0, 0.0, angleChangle);
////			SmartDashboard::PutNumber("Angle to Gear", aimer.TwoCameraAngleFilter()); //TODO: change if we use something else for the cameras
////			frc::Wait(.01);
////			failsafe++;
////			bigFailsafe++;
////		}
////		failsafe = 0;
////		while (gyro.GetYaw() + aimer.TwoCameraAngleFilter() >= desiredAngle - 3 && gyro.GetYaw() + aimer.TwoCameraAngleFilter() <= desiredAngle + 3 && failsafe < 200 && !IsOperatorControl())
////		{
////			angleChangle = pid.PIDAngle(gyro.GetYaw(), aimer.TwoCameraAngleFilter() + gyro.GetYaw() + 20 * sign);
////			robotDrive.MecanumDrive_Cartesian(0, -0.5, angleChangle);
////			SmartDashboard::PutNumber("Angle to Gear", aimer.TwoCameraAngleFilter());
////			frc::Wait(.01);
////			failsafe++;
////		}
////		failsafe = 0;
////		while (!(currentAngle >= desiredAngle) && failsafe < 100 && !IsOperatorControl())
////		{
////			currentAngle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw(); //current angle
////			angleChangle = pid.PIDAngle(gyro.GetYaw(), aimer.TwoCameraAngleFilter() + gyro.GetYaw());
////			robotDrive.MecanumDrive_Cartesian(0, 0.0, angleChangle);
////			SmartDashboard::PutNumber("Angle to Gear", aimer.TwoCameraAngleFilter());
////			frc::Wait(.01);
////			failsafe++;
////		}
////		failsafe = 0;
////	}
////	 */
////	/*while(/*filter.ultrasonicFilter(leftUltrasonic, rightUltrasonic) > 12 && *//*failsafe < 200 && !IsOperatorControl())
////	{
////		float angleChangle = pid.PIDAngle(gyro.GetYaw(), aimer.TwoCameraAngleFilter() + gyro.GetYaw()); //TODO: change to joseph's accumulator code
////		float driveSpeed = 0.5 - ((getAverageDistance(leftProx, rightProx) / 80) - 0.5);
////		robotDrive.MecanumDrive_Cartesian(0, -driveSpeed, angleChangle);
////		SmartDashboard::PutNumber("Angle to Gear", aimer.TwoCameraAngleFilter());
////		SmartDashboard::PutNumber("leftProx", leftUltrasonic);
////		SmartDashboard::PutNumber("rightProx", rightUltrasonic);
////		SmartDashboard::PutNumber("leftIR", leftIR.get());
////		SmartDashboard::PutNumber("rightIR", rightIR.get());
////		//SmartDashboard::PutNumber("Ultrasonic Filter", filter.ultrasonicFilter(leftUltrasonic, rightUltrasonic));
////		frc::Wait(.01);
////		failsafe++;
////	}*/
////
////	//TODO: drop gear, or pretend to
////	gear.setBottom(false); //TODO: may need to switch to true
////	frc::Wait(.5);
////	//Woa, woa, woa, back it up
////	failsafe = 0;
////	isDone = false;
////	while (!isDone && failsafe < 100 && !IsOperatorControl() && IsEnabled()) { //TODO: change failsafe < 200 to be the calculated value to go the desired distance
////		currentAngle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw(); //current angle
////		angleChangle = pid.PIDAngle(currentAngle, 0.0); //drive straight
////		robotDrive.MecanumDrive_Cartesian(0.0, Constants::minForwardPower, angleChangle); //drive straight (backwards) for a bit
////		frc::Wait(.01);
////		failsafe++;
////	}
////	failsafe = 0;
////	isDone = false;
////	//find boilerino
////	//Turn until you can see the target(FOREVER)
////	while(aimer.GetBoilerAge() > 1 && failsafe <= 500 && !IsOperatorControl() && IsEnabled())
////	{
////		robotDrive.MecanumDrive_Cartesian(0, 0, 0.31415926);
////		failsafe++;
////		frc::Wait(.01);
////	}
////	robotDrive.MecanumDrive_Cartesian(0.0, 0.0, 0.0);
////	//turn to boiling point
////	failsafe = 0;
////	pid.resetPIDAngle(); //if loop is done reset values
////	float angle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw();
////	float angleOutput;
////	float desireAngle = aimer.GetBoilerAngle() + angle;
////	desireAngle = desireAngle < 0 ? desireAngle + 360 : desireAngle > 360 ? desireAngle - 360 : desireAngle; //nested question mark operator - bada bop bop ba, I'm lovin' it - TODO: change to whatever we're actually using for the camera
////	while(failsafe < 500 && fabs(desireAngle - angle) > 1 && !IsOperatorControl() && IsEnabled()) //turn to boiler
////	{
////		angle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw();
////		angleOutput = pid.PIDAngle(angle, desireAngle);
////		robotDrive.MecanumDrive_Cartesian(0,0,angleOutput);
////		failsafe++;
////		frc::Wait(.01);
////	}
////	failsafe = 0;
////	shooter.shoot(.5); //TODO: change shooter speed
////	//We should have it be stuck in a while loop until all of the balls have been shot.  We should tell the electrical team to put a sensor on the robot to tell us if we're able to shoot anymore.
////	int theVariableThatTellsTheProgramTheDistanceToTheBoilerFromHopper = 55;
////
////	while(aimer.GetBoilerDistance() > theVariableThatTellsTheProgramTheDistanceToTheBoilerFromHopper + 10 || aimer.GetBoilerDistance() < theVariableThatTellsTheProgramTheDistanceToTheBoilerFromHopper - 10 && failsafe < 500 && !IsOperatorControl() && IsEnabled())
////	{
////		float sanicSpead;
////		if(aimer.GetBoilerDistance() < theVariableThatTellsTheProgramTheDistanceToTheBoilerFromHopper)
////		{
////			sanicSpead = -.2;
////		}else sanicSpead = .2;
////		//might hafta go fest
////		if(aimer.GetBoilerDistance() < theVariableThatTellsTheProgramTheDistanceToTheBoilerFromHopper - 20 || aimer.GetBoilerDistance() > theVariableThatTellsTheProgramTheDistanceToTheBoilerFromHopper + 20)
////			sanicSpead *= 2;
////		robotDrive.MecanumDrive_Cartesian(0,sanicSpead,0);
////		failsafe++;
////		frc::Wait(.01);
////	}
////	failsafe = 0;
////	robotDrive.MecanumDrive_Cartesian(0,0,0);
////	failsafe = 0;
////	pid.resetPIDAngle(); //if loop is done reset values
////	angle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw();
////	desireAngle = 180 + angle;
////	desireAngle = desireAngle < 0 ? desireAngle + 360 : desireAngle > 360 ? desireAngle - 360 : desireAngle; //nested question mark operator - bada bop bop ba, I'm lovin' it - TODO: change to whatever we're actually using for the camera
////	while(failsafe < 500 && fabs(desireAngle - angle) > 1 && !IsOperatorControl() && IsEnabled()) //Make sho you're zeroed to hero
////	{
////		angle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw();
////		angleOutput = pid.PIDAngle(angle, desireAngle);
////		robotDrive.MecanumDrive_Cartesian(0,0,angleOutput);
////		failsafe++;
////		frc::Wait(.01);
////	}
////	failsafe = 0;
////	robotDrive.MecanumDrive_Cartesian(0,0,0);
////	//are there sideways ultra sanics?
////	float distanceToWall = /*Get distance to hopper from left ultra sonic.  Convert to inches if needed*/ 0;
////	encoder->Reset();
////	while(encoder->GetDistance() < distanceToWall - 10 && !IsOperatorControl() && IsEnabled())
////	{
////		robotDrive.MecanumDrive_Cartesian(0, 0.5, 0);
////	}
////	robotDrive.MecanumDrive_Cartesian(0,0,0);
////	shooter.shoot(.5); //TODO: change shooter speed
//	//robotDrive.SetSafetyEnabled(true);
//}

void Robot::Autonomous() {
	//This is self-explanatory
	robotDrive.SetSafetyEnabled(false);
	gyro.ResetDisplacement();
	gyro.ZeroYaw();
	xEncoder.Reset();
	yEncoder.Reset();
	Accumulator accum((float)0.0, (float)0.0, (float)0.0, 2, leftProx, rightProx, aimer, xEncoder, yEncoder, pid);//Accumulatpr constructor
	int failsafe = 0;
	float desiredAngle = 0.0;//This is the angle to which the robot will try to aim
	float angleChangle = 0.0;
	float yOutput = 0.0;//I don't even know
	float currentAngle = 0.0;
	float leftUltrasonic = leftProx.GetRangeInches();
	float rightUltrasonic = rightProx.GetRangeInches();
	bool isDone = false, started = false;
	float yEncoderStartPos = yEncoder.GetDistance();
	Timer autoTimer;
	Wait(.5); //TODO: untested code - uh oh spaghettios

	SmartDashboard::PutBoolean("AutoMode 1 Entered", false);
	SmartDashboard::PutBoolean("AutoMode 2 Entered", false);
	SmartDashboard::PutBoolean("AutoMode 3 Entered", false);

	leftProx.SetAutomaticMode(true);
	rightProx.SetAutomaticMode(true);
	pid.setAngle(SmartDashboard::GetNumber("angle_p", Constants::angle_p_default), SmartDashboard::GetNumber("angle_i", Constants::angle_i_default), SmartDashboard::GetNumber("angle_d", Constants::angle_d_default));
	pid.setY(SmartDashboard::GetNumber("y_p", Constants::y_p_default), SmartDashboard::GetNumber("y_i", Constants::y_i_default), SmartDashboard::GetNumber("y_d", Constants::y_d_default));
	pid.setX(SmartDashboard::GetNumber("x_p", Constants::x_p_default), SmartDashboard::GetNumber("x_i", Constants::x_i_default), SmartDashboard::GetNumber("x_d", Constants::x_d_default));
	gyro.Reset();
	gyro.ResetDisplacement();


	int time = (SmartDashboard::GetBoolean("Is Red", true)) ? 132 : 149;
	float driveX = 0.001, driveY = 0.001, driveZ = 0.001;
	currentAngle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw();
	DoubleDouble driveVals = accum.drive(started, true, false, false, 4, desiredAngle, 2, currentAngle);//This is under the impression that the encoders are moving positive and the motors but the robot distance from the peg is decreasing
	switch((int)SmartDashboard::GetNumber("Starting Position", 3))//This gets the starting position from the user
	{
	case 1://Position 1: straight from the middle peg
		SmartDashboard::PutBoolean("AutoMode 1 Entered", true);
		SmartDashboard::PutString("Auto Step", "Case 1 Entered");
		//I think we should just be able to go straight from here, dude
		desiredAngle = 0.0;//We want to go straight ahead
		isDone = false;
		started = false;
		autoTimer.Start();
		autoTimer.Reset();
		while ((leftUltrasonic != 0 || rightUltrasonic != 0) && failsafe < 700 && !IsOperatorControl() && autoTimer.Get() < 5) {//This makes sure that one of the ultrasonics is working
			SmartDashboard::PutString("Auto Step", "Loop 1 entered");
			currentAngle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw();
			driveVals = accum.drive(started, true, false, false, 4, desiredAngle, 2, currentAngle); //This is under the impression that the encoders are moving positive and the motors but the robot distance from the peg is decreasing
			started = true;
			driveX = driveVals.x;
			driveY = fabs(driveVals.y);
			driveZ = driveVals.angle;
			robotDrive.MecanumDrive_Cartesian(driveX, driveY, driveZ); //drive forward while staying straight
			failsafe++;
			leftUltrasonic = leftProx.GetRangeInches();
			rightUltrasonic = rightProx.GetRangeInches();
			if (driveX < .01 && driveY < .01 && driveZ < .01 && (leftUltrasonic < 8 || rightUltrasonic < 8)) {
				failsafe = 701;
			}
			Wait(.01);
			SmartDashboard::PutNumber("Auto Timer", autoTimer.Get());
			/*if (filter.ultrasonicFilter(leftUltrasonic, rightUltrasonic) < 12) { //TODO: change when we get the kalman filters on
				isDone = true; //stop the loop
				failsafe = 400;
			}*/
			/*if (driveY == 0 && driveX == 0)
				isDone = true;
			frc::Wait(.01); //wait to avoid hogging cpu cycles
			failsafe++; //increment failsafe variable
		}
		robotDrive.MecanumDrive_Cartesian(0.0, 0.0, 0.0); //stop robot
		failsafe = 0; //reset failsafe for use later*/
		}
		SmartDashboard::PutNumber("leftProx", leftProx.GetRangeInches());
		SmartDashboard::PutNumber("rightProx", rightProx.GetRangeInches());
		leftUltrasonic = leftProx.GetRangeInches();
		rightUltrasonic = rightProx.GetRangeInches();
		if (leftUltrasonic == 0 && rightUltrasonic == 0)//This kablamos the gear if the ultrasonics suck
		{
			SmartDashboard::PutString("Auto Step", "Loop 2 Entered - broken ultrasonics");
			autoTimer.Reset();
			while (autoTimer.Get() < 2) {
				currentAngle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw();
				angleChangle = pid.PIDAngle(currentAngle, 0.0);
				robotDrive.MecanumDrive_Cartesian(0.0, .5 - autoTimer.Get() * .2, angleChangle);
				Wait(.01);
			}
			robotDrive.MecanumDrive_Cartesian(0.0, 0.0, 0.0);
		}
		else if (leftProx.GetRangeInches() < 10 && rightProx.GetRangeInches() < 10 && (gyro.GetYaw() > 355 || gyro.GetYaw() < 5) ) {
			SmartDashboard::PutString("Auto Step", "gear open close loop entered");
			gear.setBottom(true);
			Wait(1.0);
			gear.setPusher(true);
			Wait(1.5);
			robotDrive.MecanumDrive_Cartesian(0.0, -.5, 0.0);
			Wait(1.0);
			robotDrive.MecanumDrive_Cartesian(0, 0, 0);
			SmartDashboard::PutString("Auto Step", "case 1 done");
			gear.setBottom(false);
		}
		robotDrive.MecanumDrive_Cartesian(0.0, 0.0, 0.0);
		gear.setPusher(false);
		Wait(.5);
		gear.setBottom(false);
		break;
	case 2://Position 2: on the left
		SmartDashboard::PutBoolean("AutoMode 2 Entered", true);
		desiredAngle = 60;//We want to turn right 60 degrees
		isDone = false;
		started = false;
		//while(yEncoder.GetDistance() < 90)
		//time = (SmartDashboard::GetBoolean("Is Red", true)) ? 132 : 149;//This is the time for the going straight part
		time = 2.1;
		autoTimer.Start();
		autoTimer.Reset();
		while (autoTimer.Get() < time && (yEncoder.GetDistance() - yEncoderStartPos) < 91) {
			robotDrive.MecanumDrive_Cartesian(0, 0.4, pid.PIDAngle(gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw(), 0));
			frc::Wait(.01);
		}
		robotDrive.MecanumDrive_Cartesian(0.0, 0.0, 0.0);
		autoTimer.Reset();
		while(fabs(gyro.GetYaw() - desiredAngle) > 2 && failsafe < 500 && IsAutonomous() && !IsOperatorControl() && IsEnabled() && autoTimer.Get() < 5) {//Turn to the angle
			robotDrive.MecanumDrive_Cartesian(0, 0, pid.PIDAngle(gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw(), desiredAngle));
			failsafe++;
			Wait(.01);
		}
		failsafe = 0;
		frc::Wait(.5);
		autoTimer.Reset();
		while ((leftUltrasonic != 0 || rightUltrasonic != 0) && failsafe < 700 && !IsOperatorControl() && IsEnabled() && autoTimer.Get() < 5) {//This makes sure that one of the ultrasonics is working
					currentAngle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw();
					driveVals = accum.drive(started, true, false, false, 4, desiredAngle, 2, currentAngle);//This is under the impression that the encoders are moving positive and the motors but the robot distance from the peg is decreasing
					started = true;
					driveX = driveVals.x;
					driveY = fabs(driveVals.y);
					driveZ = driveVals.angle;
					robotDrive.MecanumDrive_Cartesian(driveX, driveY, driveZ); //drive forward while staying straight
					failsafe++;
					leftUltrasonic = leftProx.GetRangeInches();
					rightUltrasonic = rightProx.GetRangeInches();
					if (driveX < .01 && driveY < .01 && driveZ < .01 && (leftUltrasonic < 8 || rightUltrasonic < 8)) {
						failsafe = 701;
					}
					Wait(.01);
					SmartDashboard::PutNumber("Auto Timer", autoTimer.Get());
					/*if (filter.ultrasonicFilter(leftUltrasonic, rightUltrasonic) < 12) { //TODO: change when we get the kalman filters on
						isDone = true; //stop the loop
						failsafe = 400;
					}*/
					/*if (driveY == 0 && driveX == 0)
						isDone = true;
					frc::Wait(.01); //wait to avoid hogging cpu cycles
					failsafe++; //increment failsafe variable
				}
				robotDrive.MecanumDrive_Cartesian(0.0, 0.0, 0.0); //stop robot
				failsafe = 0; //reset failsafe for use later*/
				}
				robotDrive.MecanumDrive_Cartesian(0.0, 0.0, 0.0);
				SmartDashboard::PutNumber("leftProx", leftProx.GetRangeInches());
				SmartDashboard::PutNumber("rightProx", rightProx.GetRangeInches());
				currentAngle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw();
//				if (leftUltrasonic == 0 && rightUltrasonic == 0)//This kablamos the gear if the ultrasonics suck
//				{
//					robotDrive.MecanumDrive_Cartesian(0, .4, 0);
//					frc::Wait(1.5);
//					robotDrive.MecanumDrive_Cartesian(0, 0, 0);
//				}
				if (leftProx.GetRangeInches() < 10 && rightProx.GetRangeInches() < 10 && fabs(desiredAngle - currentAngle) < 5) {
					SmartDashboard::PutString("Auto Step", "gear open close");
					gear.setBottom(true);
					Wait(1.0);
					gear.setPusher(true);
					Wait(1.5);
					robotDrive.MecanumDrive_Cartesian(0.0, -.5, 0.0);
					Wait(1.0);
					robotDrive.MecanumDrive_Cartesian(0, 0, 0);
					SmartDashboard::PutString("Auto Step", "case 1 done");
					gear.setBottom(false);
				}
				robotDrive.MecanumDrive_Cartesian(0.0, 0.0, 0.0);
		break;
	case 3://Position 3: on the right
		SmartDashboard::PutBoolean("AutoMode 3 Entered", true);
		//time = (SmartDashboard::GetBoolean("Is Red", true)) ? 149 : 132;
		time = 2.0;
		desiredAngle = 300;
		isDone = false;
		started = false;
		autoTimer.Start();
		autoTimer.Reset();
		//while(yEncoder.GetDistance() < 90)
		while (autoTimer.Get() < time && (yEncoder.GetDistance() - yEncoderStartPos) < 91) {
			currentAngle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw();
			robotDrive.MecanumDrive_Cartesian(0, 0.4, pid.PIDAngle(currentAngle, 0));
			frc::Wait(.01);
		}
		robotDrive.MecanumDrive_Cartesian(0.0, 0.0, 0.0);
		failsafe = 0;
		autoTimer.Reset();
		currentAngle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw();
		while(fabs(currentAngle - desiredAngle) > 2 && failsafe < 500 && IsAutonomous() && !IsOperatorControl() && IsEnabled() && autoTimer.Get() < 5) {
			currentAngle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw();
			robotDrive.MecanumDrive_Cartesian(0, 0, pid.PIDAngle(currentAngle, desiredAngle));
			failsafe++;
			Wait(.01);
		}
		robotDrive.MecanumDrive_Cartesian(0.0, 0.0, 0.0);
		failsafe = 0;
		frc::Wait(.5);
		autoTimer.Reset();
		while ((leftUltrasonic != 0 || rightUltrasonic != 0) && failsafe < 700 && !IsOperatorControl() && IsEnabled() && autoTimer.Get() < 7) {//This makes sure that one of the ultrasonics is working
					currentAngle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw();
					driveVals = accum.drive(started, true, false, false, 4, desiredAngle, 2, currentAngle);//This is under the impression that the encoders are moving positive and the motors but the robot distance from the peg is decreasing
					started = true;
					driveX = driveVals.x;
					driveY = fabs(driveVals.y);
					driveZ = driveVals.angle;
					robotDrive.MecanumDrive_Cartesian(driveX, driveY, driveZ); //drive forward while staying straight
					failsafe++;
					leftUltrasonic = leftProx.GetRangeInches();
					rightUltrasonic = rightProx.GetRangeInches();
					if (driveX < .01 && driveY < .01 && driveZ < .01 && (leftUltrasonic < 8 || rightUltrasonic < 8)) {
						failsafe = 701;
					}
					Wait(.01);
					SmartDashboard::PutNumber("Auto Timer", autoTimer.Get());
					/*if (filter.ultrasonicFilter(leftUltrasonic, rightUltrasonic) < 12) { //TODO: change when we get the kalman filters on
						isDone = true; //stop the loop
						failsafe = 400;
					}*/
					/*if (driveY == 0 && driveX == 0)
						isDone = true;
					frc::Wait(.01); //wait to avoid hogging cpu cycles
					failsafe++; //increment failsafe variable
				}
				robotDrive.MecanumDrive_Cartesian(0.0, 0.0, 0.0); //stop robot
				failsafe = 0; //reset failsafe for use later*/
				}
				robotDrive.MecanumDrive_Cartesian(0.0, 0.0, 0.0);
				SmartDashboard::PutNumber("leftProx", leftProx.GetRangeInches());
				SmartDashboard::PutNumber("rightProx", rightProx.GetRangeInches());
				currentAngle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw();
//				if (leftUltrasonic == 0 && rightUltrasonic == 0)//This kablamos the gear if the ultrasonics suck
//				{
//					robotDrive.MecanumDrive_Cartesian(0, .4, 0);
//					frc::Wait(1.5);
//					robotDrive.MecanumDrive_Cartesian(0, 0, 0);
//				}
				if (leftProx.GetRangeInches() < 10 && rightProx.GetRangeInches() < 10 && fabs(desiredAngle - currentAngle) < 5 ) {
					SmartDashboard::PutString("Auto Step", "gear open close");
					gear.setBottom(true);
					Wait(1.0);
					gear.setPusher(true);
					Wait(1.5);
					robotDrive.MecanumDrive_Cartesian(0.0, -.5, 0.0);
					Wait(1.0);
					robotDrive.MecanumDrive_Cartesian(0, 0, 0);
					SmartDashboard::PutString("Auto Step", "case 1 done");
					gear.setBottom(false);
				}
				robotDrive.MecanumDrive_Cartesian(0.0, 0.0, 0.0);
		break;
	case 4: //cross the line
		autoTimer.Start();
		autoTimer.Reset();
		while (autoTimer.Get() < 4) {
			currentAngle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw();
			angleChangle = pid.PIDAngle(currentAngle, 0.0);
			robotDrive.MecanumDrive_Cartesian(0.0, .2, angleChangle);
		}
		robotDrive.MecanumDrive_Cartesian(0.0, 0.0, 0.0);


	/*case 4:
		angleChangle = 0.0;
		currentAngle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw();
		autoTimer.Start();
		autoTimer.Reset();
		while (autoTimer.Get() < 1.5) {
			currentAngle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw(); //TODO: swap front back
			angleChangle = pid.PIDAngle(currentAngle, 0.0);
			robotDrive.MecanumDrive_Cartesian(0.0, 1.0, 0.0);
			SmartDashboard::PutNumber("autoTimer", autoTimer.Get());
			Wait(.01);
		}
		robotDrive.MecanumDrive_Cartesian(0.0, 0.0, 0.0);
		break;

	case 5:
	default:
		robotDrive.MecanumDrive_Cartesian(0.0, 1.0, 0.0);
		Wait(1.5);
		robotDrive.MecanumDrive_Cartesian(0.0, 0.0, 0.0);
		break;*/
	}

//	if (leftUltrasonic < 7 && leftUltrasonic != 0 || rightUltrasonic < 7 && rightUltrasonic != 0)
//	{
//		gear.setBottom(!gear.getBottom());
//		frc::Wait(.5);
//		robotDrive.MecanumDrive_Cartesian(0, -Constants::minForwardPower, 0);
//		frc::Wait(3);
//		robotDrive.MecanumDrive_Cartesian(0, 0, 0);
//	}




//	float bigFailsafe = 0;
//	failsafe = 0;
//	DoubleDouble driveVals(0, 0, 0);
//	while(failsafe < 500 && driveVals.x > 0.5 && !IsOperatorControl() && IsEnabled())
//	{
//		driveVals = accum.drive(true, true, false, false, 12, 0, 2, gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw());
//		robotDrive.MecanumDrive_Cartesian(driveVals.x, driveVals.y, driveVals.angle);
//		failsafe++;
//	}
//	/*while (!(fabs(aimer.TwoCameraAngleFilter()) <= 3.0) && bigFailsafe < 500 && !IsOperatorControl())//This adjusts the accuracy of the aiming of the robot TODO: change to whatever we're actually using for the camera
//	{
//		int sign = (aimer.TwoCameraAngleFilter() < 0) ? -1 : 1;//Which direction to turn
//		while (fabs(currentAngle - desiredAngle) < 3 && failsafe < 100 && !IsOperatorControl())
//		{
//			currentAngle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw(); //current angle
//			desiredAngle = aimer.TwoCameraAngleFilter() + currentAngle;
//			desiredAngle = desiredAngle < 0 ? desiredAngle + 360 : desiredAngle > 360 ? desiredAngle - 360 : desiredAngle; //nested question mark operator - bada bop bop ba, I'm lovin' it - TODO: change to whatever we're actually using for the camera
//			float angleChangle = pid.PIDAngle(currentAngle, desiredAngle);
//			robotDrive.MecanumDrive_Cartesian(0.0, 0.0, angleChangle);
//			SmartDashboard::PutNumber("Angle to Gear", aimer.TwoCameraAngleFilter()); //TODO: change if we use something else for the cameras
//			frc::Wait(.01);
//			failsafe++;
//			bigFailsafe++;
//		}
//		failsafe = 0;
//		while (gyro.GetYaw() + aimer.TwoCameraAngleFilter() >= desiredAngle - 3 && gyro.GetYaw() + aimer.TwoCameraAngleFilter() <= desiredAngle + 3 && failsafe < 200 && !IsOperatorControl())
//		{
//			angleChangle = pid.PIDAngle(gyro.GetYaw(), aimer.TwoCameraAngleFilter() + gyro.GetYaw() + 20 * sign);
//			robotDrive.MecanumDrive_Cartesian(0, -0.5, angleChangle);
//			SmartDashboard::PutNumber("Angle to Gear", aimer.TwoCameraAngleFilter());
//			frc::Wait(.01);
//			failsafe++;
//		}
//		failsafe = 0;
//		while (!(currentAngle >= desiredAngle) && failsafe < 100 && !IsOperatorControl())
//		{
//			currentAngle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw(); //current angle
//			angleChangle = pid.PIDAngle(gyro.GetYaw(), aimer.TwoCameraAngleFilter() + gyro.GetYaw());
//			robotDrive.MecanumDrive_Cartesian(0, 0.0, angleChangle);
//			SmartDashboard::PutNumber("Angle to Gear", aimer.TwoCameraAngleFilter());
//			frc::Wait(.01);
//			failsafe++;
//		}
//		failsafe = 0;
//	}
//	 */
//	/*while(/*filter.ultrasonicFilter(leftUltrasonic, rightUltrasonic) > 12 && *//*failsafe < 200 && !IsOperatorControl())
//	{
//		float angleChangle = pid.PIDAngle(gyro.GetYaw(), aimer.TwoCameraAngleFilter() + gyro.GetYaw()); //TODO: change to joseph's accumulator code
//		float driveSpeed = 0.5 - ((getAverageDistance(leftProx, rightProx) / 80) - 0.5);
//		robotDrive.MecanumDrive_Cartesian(0, -driveSpeed, angleChangle);
//		SmartDashboard::PutNumber("Angle to Gear", aimer.TwoCameraAngleFilter());
//		SmartDashboard::PutNumber("leftProx", leftUltrasonic);
//		SmartDashboard::PutNumber("rightProx", rightUltrasonic);
//		SmartDashboard::PutNumber("leftIR", leftIR.get());
//		SmartDashboard::PutNumber("rightIR", rightIR.get());
//		//SmartDashboard::PutNumber("Ultrasonic Filter", filter.ultrasonicFilter(leftUltrasonic, rightUltrasonic));
//		frc::Wait(.01);
//		failsafe++;
//	}*/
//
//	//TODO: drop gear, or pretend to
//	gear.setBottom(false); //TODO: may need to switch to true
//	frc::Wait(.5);
//	//Woa, woa, woa, back it up
//	failsafe = 0;
//	isDone = false;
//	while (!isDone && failsafe < 100 && !IsOperatorControl() && IsEnabled()) { //TODO: change failsafe < 200 to be the calculated value to go the desired distance
//		currentAngle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw(); //current angle
//		angleChangle = pid.PIDAngle(currentAngle, 0.0); //drive straight
//		robotDrive.MecanumDrive_Cartesian(0.0, Constants::minForwardPower, angleChangle); //drive straight (backwards) for a bit
//		frc::Wait(.01);
//		failsafe++;
//	}
//	failsafe = 0;
//	isDone = false;
//	//find boilerino
//	//Turn until you can see the target(FOREVER)
//	while(aimer.GetBoilerAge() > 1 && failsafe <= 500 && !IsOperatorControl() && IsEnabled())
//	{
//		robotDrive.MecanumDrive_Cartesian(0, 0, 0.31415926);
//		failsafe++;
//		frc::Wait(.01);
//	}
//	robotDrive.MecanumDrive_Cartesian(0.0, 0.0, 0.0);
//	//turn to boiling point
//	failsafe = 0;
//	pid.resetPIDAngle(); //if loop is done reset values
//	float angle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw();
//	float angleOutput;
//	float desireAngle = aimer.GetBoilerAngle() + angle;
//	desireAngle = desireAngle < 0 ? desireAngle + 360 : desireAngle > 360 ? desireAngle - 360 : desireAngle; //nested question mark operator - bada bop bop ba, I'm lovin' it - TODO: change to whatever we're actually using for the camera
//	while(failsafe < 500 && fabs(desireAngle - angle) > 1 && !IsOperatorControl() && IsEnabled()) //turn to boiler
//	{
//		angle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw();
//		angleOutput = pid.PIDAngle(angle, desireAngle);
//		robotDrive.MecanumDrive_Cartesian(0,0,angleOutput);
//		failsafe++;
//		frc::Wait(.01);
//	}
//	failsafe = 0;
//	shooter.shoot(.5); //TODO: change shooter speed
//	//We should have it be stuck in a while loop until all of the balls have been shot.  We should tell the electrical team to put a sensor on the robot to tell us if we're able to shoot anymore.
//	int theVariableThatTellsTheProgramTheDistanceToTheBoilerFromHopper = 55;
//
//	while(aimer.GetBoilerDistance() > theVariableThatTellsTheProgramTheDistanceToTheBoilerFromHopper + 10 || aimer.GetBoilerDistance() < theVariableThatTellsTheProgramTheDistanceToTheBoilerFromHopper - 10 && failsafe < 500 && !IsOperatorControl() && IsEnabled())
//	{
//		float sanicSpead;
//		if(aimer.GetBoilerDistance() < theVariableThatTellsTheProgramTheDistanceToTheBoilerFromHopper)
//		{
//			sanicSpead = -.2;
//		}else sanicSpead = .2;
//		//might hafta go fest
//		if(aimer.GetBoilerDistance() < theVariableThatTellsTheProgramTheDistanceToTheBoilerFromHopper - 20 || aimer.GetBoilerDistance() > theVariableThatTellsTheProgramTheDistanceToTheBoilerFromHopper + 20)
//			sanicSpead *= 2;
//		robotDrive.MecanumDrive_Cartesian(0,sanicSpead,0);
//		failsafe++;
//		frc::Wait(.01);
//	}
//	failsafe = 0;
//	robotDrive.MecanumDrive_Cartesian(0,0,0);
//	failsafe = 0;
//	pid.resetPIDAngle(); //if loop is done reset values
//	angle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw();
//	desireAngle = 180 + angle;
//	desireAngle = desireAngle < 0 ? desireAngle + 360 : desireAngle > 360 ? desireAngle - 360 : desireAngle; //nested question mark operator - bada bop bop ba, I'm lovin' it - TODO: change to whatever we're actually using for the camera
//	while(failsafe < 500 && fabs(desireAngle - angle) > 1 && !IsOperatorControl() && IsEnabled()) //Make sho you're zeroed to hero
//	{
//		angle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw();
//		angleOutput = pid.PIDAngle(angle, desireAngle);
//		robotDrive.MecanumDrive_Cartesian(0,0,angleOutput);
//		failsafe++;
//		frc::Wait(.01);
//	}
//	failsafe = 0;
//	robotDrive.MecanumDrive_Cartesian(0,0,0);
//	//are there sideways ultra sanics?
//	float distanceToWall = /*Get distance to hopper from left ultra sonic.  Convert to inches if needed*/ 0;
//	encoder->Reset();
//	while(encoder->GetDistance() < distanceToWall - 10 && !IsOperatorControl() && IsEnabled())
//	{
//		robotDrive.MecanumDrive_Cartesian(0, 0.5, 0);
//	}
//	robotDrive.MecanumDrive_Cartesian(0,0,0);
//	shooter.shoot(.5); //TODO: change shooter speed
//	robotDrive.SetSafetyEnabled(true);
}


inline float getAverageDistance(const Ultrasonic& leftProx, const Ultrasonic& rightProx)
{
	return ((float)leftProx.GetRangeInches() + (float)rightProx.GetRangeInches()) / 2.0;
}

START_ROBOT_CLASS(Robot)
