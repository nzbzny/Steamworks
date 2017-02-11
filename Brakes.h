#include "WPILib.h"
#include "Pneumatics.h"
#include "Constants.h"

#ifndef SRC_BRAKES_H
#define SRC_BRAKES_H

class Brakes {

	Pneumatics brake;
public:
	Brakes(int brakeInSole, int brakeOutSole);
	void set(bool state);
	bool get();
};

#endif
