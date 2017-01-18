/*
 * Position.cpp
 *
 *  Created on: Jan 14, 2017
 *      Author: Owner
 */
#include "Position.h"


using namespace std;


Coordinates Position::getRawCenter() const
{

	float centerX = navX.GetDisplacementX() * 39.307 + Constants::navXDistance * std::cos(Constants::navXAngle + angleOffset) + initial.x;
	float centerY = navX.GetDisplacementY() * 39.307 - Constants::navXDistance * std::cos(Constants::navXAngle + angleOffset)+ initial.y;
	Coordinates tempC(centerX, centerY);
	return tempC;

}


Coordinates Position::getCenter() const
{

	Coordinates raw = getRawCenter();
	float centerX = raw.x + xOffset;
	float centerY = raw.y + yOffset;
	Coordinates center(centerX, centerY);
	return center;

}


void Position::updateWithBoiler(const float& angle, const float& range)
{

	float trueAngle = (getAngle() + angle >= 360) ? getAngle() + angle - 360 : getAngle() + angle;
	bool blueBoiler = trueAngle >= 270 || trueAngle <= 90;
	float cameraX = 0.0, cameraY = 0.0;

	if (blueBoiler)
	{

		cameraX = Constants::blueBoilerX + range * std::sin(trueAngle);
		cameraY = Constants::blueBoilerY - range * std::cos(trueAngle);

	}
	else
	{

		cameraX = Constants::redBoilerX + range * sin(trueAngle);
		cameraY = Constants::redBoilerY - range * std::cos(trueAngle);

	}

	float cameraAngleFromCenter = (Constants::cameraAngle + getAngle() >= 360) ? Constants::cameraAngle + getAngle() - 360 : Constants::cameraAngle + getAngle();
	float robotCenterX = cameraX + Constants::cameraDistance * std::cos(cameraAngleFromCenter);
	float robotCenterY = cameraY - Constants::cameraDistance * std::cos(cameraAngleFromCenter);

	Coordinates tempRaw = getRawCenter();
	xOffset = robotCenterX - tempRaw.x;
	yOffset = robotCenterY - tempRaw.y;

	long int* temp = NULL;

	fixed = time(temp);

}


Compiled Position::getAll() const
{

	Compiled all;
	long int* temp = NULL;
	Coordinates center = getCenter();
	all.position = center;
	Coordinates offsets(xOffset, yOffset);
	all.navXError = offsets;
	all.timeSinceLastFix = time(temp) - fixed;
	return all;

}










