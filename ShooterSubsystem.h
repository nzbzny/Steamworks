#include "WPILib.h"
#include "CANTalon.h"
#include "Constants.h"
#include "ADXL345_I2C.h"

#ifndef SRC_SHOOTERSUBSYSTEM_H
#define SRC_SHOOTERSUBSYSTEM_H

#define PI 3.14159265

class ShooterSubsystem {

CANTalon rotator; //shooter angle
CANTalon shooter;
Talon noah; //agitator - noah (me) is the biggest agitator on the team
ADXL345_I2C accel;

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
	float getEncoder();
	float Roll();
	float Pitch();
};

#endif
