#include "WPILib.h"
#include "CANTalon.h"

#ifndef SRC_SHOOTERSUBSYSTEM_H
#define SRC_SHOOTERSUBSYSTEM_H

class ShooterSubsystem {



public:
	ShooterSubsystem();
	void move(float moveValue);
	void setAngle(float angle);
	void setSpeed(float speed);
};

#endif
