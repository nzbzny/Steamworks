#include "WPILib.h"
#include "Constants.h"

#ifndef SRC_INTAKESUBSYSTEM_H
#define SRC_INTAKESUBSYSTEM_H

class IntakeSubsystem {

	Talon intake;
	Talon verticalConveyor;

public:
	IntakeSubsystem(int intakePin, int verticalConveyorPin);
	void runIntake(float speed);
	void runVerticalConveyor(float speed);


};

#endif
