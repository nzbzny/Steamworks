#include "ShooterSubsystem.h"

ShooterSubsystem::ShooterSubsystem(int rotatorChannel, int shooterChannel) :
  rotator(rotatorChannel),
  shooter(shooterChannel)
{
  rotator.SetTalonControlMode(CANTalon::TalonControlMode::kPercentVbus); //enum might be wrong - check in eclipse
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

void ShooterSubsystem::move(float moveValue) { //rotate the shooter at a speed (kPercentVbus)
  rotator.Set(moveValue);
}

void ShooterSubsystem::setAngle(float angle) {

}

void ShooterSubsystem::setSpeed(float speed) { //set speed to shoot the balls at
  shooter.Set(speed);
}
