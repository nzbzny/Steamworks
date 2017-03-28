#include "ShooterSubsystem.h"

ShooterSubsystem::ShooterSubsystem(int rotatorChannel, int shooterChannel, int agitatorChannel) :
  rotator(rotatorChannel),
  shooter(shooterChannel),
  noah(agitatorChannel),
  accel(I2C::Port::kOnboard)
{
  //rotator.SetTalonControlMode(CANTalon::TalonControlMode::kThrottleMode); //enum might be wrong - check in eclipse
  //shooter.SetFeedbackDevice(CANTalon::FeedbackDevice::CtreMagEncoder_Relative);
  //shooter.SetTalonControlMode(CANTalon::TalonControlMode::kSpeedMode);
  //shooter.SetTalonControlMode(CANTalon::TalonControlMode::kThrottleMode);
  //shooter.SetF(0.1);
//  shooter.SetP(0.1);
//  shooter.SetI(0.1);
//  shooter.SetD(0.1);
//  shooter.EnableControl();
	//rotator.SetTalonControlMode(CANTalon::TalonControlMode::kThrottleMode);
	//shooter.SetTalonControlMode(CANTalon::TalonControlMode::kThrottleMode);
}

void ShooterSubsystem::enable() { //enable cantalons
  rotator.Enable(); 
  shooter.Enable();
}

void ShooterSubsystem::disable() { //disable cantalons
  rotator.Disable();
  shooter.Disable();
}

void ShooterSubsystem::agitate(float speed) { //move agitator
	noah.Set(speed); //set agitator speed
}

void ShooterSubsystem::move(float moveValue) { //rotate the shooter at a speed (kPercentVbus)
  rotator.Set(moveValue); //move shooter up and down
}

float ShooterSubsystem::getAngle() {
	return Roll();
}

bool ShooterSubsystem::setAngle(float angle) { //return true when completed
  float currentAngle = getAngle();
  if (fabs(currentAngle - angle) > 1) { //if it's close enough - TODO: may have to lower acceptable error
    move(.25); //slowly angle - TODO: pid maybe? see how well it works
    return false; //not done yet
  } else {
    move(0.0); //stop moving - you're there
    return true; //done
  }
}

void ShooterSubsystem::setSpeed(float speed) { //set speed to shoot the balls at
	//shooter.Set(800);
	//shooter.Set(speed * Constants::shooterMaxSpeed); //TODO: get shooter max speed
	shooter.Set(speed);
}

void ShooterSubsystem::shoot(float speed) { //shoot the balls at a certain speed
	agitate(-1 * -1.0);
	setSpeed(speed);
}

void ShooterSubsystem::stop() { //completely stop the shooter from moving
	agitate(0.0);
	setSpeed(0.0);
	move(0.0);
}

float ShooterSubsystem::getEncoder() {
	return shooter.GetEncPosition();
}

float ShooterSubsystem::Roll() {
	return -(atan2(accel.GetX(),sqrt(accel.GetY()*accel.GetY()+accel.GetZ()*accel.GetZ()))  * 180.0) / PI;
}

float ShooterSubsystem::Pitch(){
	return (atan2(accel.GetY(),sqrt(accel.GetX()*accel.GetX()+accel.GetZ()*accel.GetZ()))  * 180.0) / PI;
}

