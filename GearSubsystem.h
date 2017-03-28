#include "WPILib.h"
#include "Pneumatics.h"

#ifndef SRC_GEARSUBSYSTEM_H
#define SRC_GEARSUBSYSTEM_H

class GearSubsystem {

	Pneumatics bottomPneumatic;
	Pneumatics gearPusher;

public:
	GearSubsystem(uint32_t bottomInSole, uint32_t bottomOutSole, uint32_t gearPusherInSole, uint32_t gearPusherOutSole);
	void setBottom(bool state);
	bool getBottom();
	void setPusher(bool state);
	bool getPusher();
};

#endif
