#include "WPILib.h"
#include "Pneumatics.h"

#ifndef SRC_GEARSUBSYSTEM_H
#define SRC_GEARSUBSYSTEM_H

class GearSubsystem {

	Pneumatics bottom;

public:
	GearSubsystem(uint32_t bottomInSole, uint32_t bottomOutSole);
	void release(bool state);
	bool getBottom();
};

#endif
