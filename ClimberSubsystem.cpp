#include "ClimberSubsystem.h"

ClimberSubsystem::ClimberSubsystem(int climberChannel) :
	climber(climberChannel) //initialize climber
{
	//climber.SetControlMode(CANTalon::ControlMode::kSpeed); //speed control mode - RPM
}

void ClimberSubsystem::enable() {
	climber.Enable();
}

void ClimberSubsystem::disable() {
	climber.Disable();
}

void ClimberSubsystem::setSpeed(float speed) {
	//climber.Set(speed * Constants::climberMaxSpeed); //TODO: temp value
	climber.Set(speed);
}
