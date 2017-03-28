#include "WPILib.h"
#include "math.h"
#include <vector>
#include <memory>
#include "Constants.h"

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
	int GetBoilerAge();
	float TwoCameraAngleFilter();
	float GetXDistanceToGear(float distance, float currentAngleError);
	float GetAngleToGear(float distance);
	float GetBoilerAngle();
	float GetBoilerDistance();
	float ConvertToZeroTo360(float rawAngle);
	float ConvertToPlusMinus180(float rawAngle);
};

#endif
