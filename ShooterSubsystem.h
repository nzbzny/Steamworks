#include "WPILib.h"
#include "CANTalon.h"

#ifndef SRC_SHOOTERSUBSYSTEM_H
#define SRC_SHOOTERSUBSYSTEM_H

class ShooterSubsystem {

CANTalon rotator;
CANTalon shooter;

public:
	ShooterSubsystem(int rotatorChannel, int shooterChannel);
	void enable();
	void disable();
	void move(float moveValue);
	void setAngle(float angle);
	void setSpeed(float speed);
};

#endif
