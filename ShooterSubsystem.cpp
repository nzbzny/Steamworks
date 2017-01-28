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

void ShooterSubsystem::agitate(float speed) {
	agitator.Set(speed);
}

void ShooterSubsystem::move(float moveValue) { //rotate the shooter at a speed (kPercentVbus)
  rotator.Set(moveValue);
}

void ShooterSubsystem::setAngle(float angle) {

}

void ShooterSubsystem::setSpeed(float speed) { //set speed to shoot the balls at
  shooter.Set(speed * Constants::shooterMaxSpeed);
}

void ShooterSubsystem::shoot(float speed) {
	agitate(.5);
	setSpeed(speed);
}

void ShooterSubsystem::stop() {
	agitate(0);
	setSpeed(0);
}

