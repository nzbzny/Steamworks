#include "WPILib.h"
#include "CANTalon.h"
#include "Constants.h"

#ifndef SRC_CLIMBERSUBSYSTEM_H
#define SRC_CLIMBERSUBSYSTEM_H

class ClimberSubsystem {

	CANTalon climber;
public:
	ClimberSubsystem(int climberChannel);
	void enable();
	void disable();
	void setSpeed(float speed);
};

#endif
