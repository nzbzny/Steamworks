#include "ShooterSubsystem.h"

ShooterSubsystem::ShooterSubsystem(int rotatorChannel, int shooterChannel, int agitatorChannel) :
  rotator(rotatorChannel),
  shooter(shooterChannel),
  agitator(agitatorChannel)
{
  rotator.SetTalonControlMode(CANTalon::TalonControlMode::kThrottleMode); //enum might be wrong - check in eclipse
  shooter.SetTalonControlMode(CANTalon::TalonControlMode::kSpeedMode); //enum might be wrong - check in eclipse
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
	agitator.Set(speed); //set agitator speed
}

void ShooterSubsystem::move(float moveValue) { //rotate the shooter at a speed (kPercentVbus)
  rotator.Set(moveValue); //move shooter up and down
}

bool ShooterSubsystem::setAngle(float angle) { //return true when completed
  float currentAngle = 0; //TODO: get current angle - find math
  if (fabs(currentAngle - angle) > 1) { //if it's close enough - TODO: may have to lower acceptable error
    move(.25); //slowly angle - TODO: pid maybe? see how well it works
    return false; //not done yet
  } else {
    move(0.0); //stop moving - you're there
    return true; //done
  }
}

void ShooterSubsystem::setSpeed(float speed) { //set speed to shoot the balls at
  shooter.Set(speed * Constants::shooterMaxSpeed); //TODO: get shooter max speed
}

void ShooterSubsystem::shoot(float speed) { //shoot the balls at a certain speed
	agitate(.5);
	setSpeed(speed);
}

void ShooterSubsystem::stop() { //completely stop the shooter from moving
	agitate(0.0);
	setSpeed(0.0);
  move(0.0);
}
