#include "WPILib.h"
#include <memory>

#ifndef SRC_AIMER_H
#define SRC_AIMER_H


class Aimer
{
public:
	Aimer();

	std::shared_ptr<NetworkTable> table;

	float GetAngleToGear();
	float GetAngleToShoot();
	float GetSpeedToShoot();
	float GetDistanceToGear();
	float GetOffset();
	int GetAge();
};

#endif
