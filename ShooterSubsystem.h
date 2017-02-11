#include "WPILib.h"
#include "CANTalon.h"
#include "Constants.h"

#ifndef SRC_SHOOTERSUBSYSTEM_H
#define SRC_SHOOTERSUBSYSTEM_H

class ShooterSubsystem {

CANTalon rotator; //shooter angle
CANTalon shooter;
Talon agitator;

public:
	ShooterSubsystem(int rotatorChannel, int shooterChannel, int agitatorChannel);
	void enable();
	void disable();
	void agitate(float speed);
	void move(float moveValue);
	float getAngle();
	bool setAngle(float angle);
	void setSpeed(float speed);
	void shoot(float speed);
	void stop();
};

#endif
