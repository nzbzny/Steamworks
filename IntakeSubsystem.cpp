/*
 * IntakeSubsystem.cpp
 *
 *  Created on: Jan 23, 2017
 *      Author: Techbrick Alt
 */
#include "IntakeSubsystem.h"

IntakeSubsystem::IntakeSubsystem(int intakePin, int verticalConveyorPin) :
	intake(intakePin),
	verticalConveyor(verticalConveyorPin)
{}

void IntakeSubsystem::runIntake(float speed) {
	intake.Set(speed);
}

void IntakeSubsystem::runVerticalConveyor(float speed) {
	verticalConveyor.Set(speed);
}
