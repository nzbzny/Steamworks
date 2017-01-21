/*
 * Position.h
 *
 *  Created on: Jan 14, 2017
 *      Author: Owner
 */
/*
#include "WPILib.h"
#include "AHRS.h"
#include "Constants.h"
#include <cmath>
#include <ctime>

#ifndef SRC_POSITION_H_
#define SRC_POSITION_H_


using namespace std;


struct Coordinates
{

	float x, y;
	Coordinates() : x(0.0), y(0.0) {}
	Coordinates(float tx, float ty) : x(tx), y(ty) {}

};


struct Compiled
{

	Coordinates position, navXError;
	time_t timeSinceLastFix;
	Compiled() : position(), navXError(), timeSinceLastFix() {}
	Compiled(float xPos, float yPos, float xError, float yError, time_t fixedFor) : position(xPos, yPos), navXError(xError, yError), timeSinceLastFix(fixedFor) {}

};


class Position
{

public:
	Position() : xOffset(0.0), yOffset(0.0), angleOffset(0.0), initial(0.0, 0.0), fixed(), navX(I2C::Port::kMXP, 200) {navX.ZeroYaw(); navX.ResetDisplacement();}
	Position(const float& x, const float& y, const float& angle) : xOffset(0.0), yOffset(0.0), angleOffset(angle), initial(x, y), fixed(), navX(I2C::Port::kMXP, 200) {navX.ZeroYaw(); navX.ResetDisplacement();}
	Position& operator=(const Position& rhs) {this->xOffset = rhs.xOffset; this->yOffset = rhs.yOffset; this->angleOffset = rhs.angleOffset; this->fixed = rhs.fixed; this->navX = rhs.navX; return *this;}
	Coordinates getCenter() const;
	Compiled getAll() const;
	void updateWithBoiler(const float& angle, const float& range);
	float getAngle() const {return fmod(((navX.GetYaw() < 0) ? navX.GetYaw() + 360 : navX.GetYaw()) + angleOffset, 360.0);}

private:
	Coordinates getRawCenter() const;
	float xOffset, yOffset, angleOffset;
	const Coordinates initial;
	time_t fixed;
	AHRS navX;

};


#endif
