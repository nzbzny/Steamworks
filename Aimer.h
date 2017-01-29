#include "WPILib.h"
#include <memory>

#ifndef SRC_AIMER_H
#define SRC_AIMER_H

#define PI 3.14159265

class Aimer
{
public:
	Aimer();

	std::shared_ptr<NetworkTable> table;

	float GetLeftAngleToGear();
	float GetRightAngleToGear();
	float GetAngleToShoot();
	float GetSpeedToShoot();
	float Distancinator();
	float GetOffset();
	int GetAge();
	void DeleteUnused();
	float twoCameraAngleFilter();
	float getXDistanceToGear();
};

#endif
