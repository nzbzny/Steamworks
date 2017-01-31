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
	intake.Set(speed); //set speed to take in balls at
}

void IntakeSubsystem::runVerticalConveyor(float speed) {
	verticalConveyor.Set(speed); //set speed of the vertical conveyor
}
