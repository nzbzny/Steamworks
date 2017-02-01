#include "ClimberSubsystem.h"

ClimberSubsystem::ClimberSubsystem(int climberChannel) :
	climber(climberChannel) //initialize climber
{
	climber.SetControlMode(CANTalon::ControlMode::kSpeed); //speed control mode - RPM
}

void ClimberSubsystem::climb() {
	climber.Set(.5 * Constants::climberMaxSpeed); //TODO: temp value
}
