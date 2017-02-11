#include "WPILib.h"
#include "math.h"
#include <vector>
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
	std::vector<double> GetLeftAngleArray();
	std::vector<double> GetRightAngleArray();
	float GetAngleToShoot();
	float GetSpeedToShoot();
	float GetDistanceToGear();
	float GetOffset();
	int GetAge();
	float TwoCameraAngleFilter();
	float GetXDistanceToGear();
	float GetAngleToGear();
};

#endif
