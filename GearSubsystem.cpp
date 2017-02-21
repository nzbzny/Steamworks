#include "GearSubsystem.h"

GearSubsystem::GearSubsystem(uint32_t bottomInSole, uint32_t bottomOutSole) :
	bottomPneumatic(bottomInSole, bottomOutSole)
{}

void GearSubsystem::setBottom(bool state) { //open / close the release mechanism
	bottomPneumatic.set(state);
}

bool GearSubsystem::getBottom() { //return state of the bottom
	return bottomPneumatic.get();
}
