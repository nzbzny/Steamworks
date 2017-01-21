#include "GearSubsystem.h"

GearSubsystem::GearSubsystem(uint32_t bottomInSole, uint32_t bottomOutSole) :
	bottom(bottomInSole, bottomOutSole)
{}

void GearSubsystem::release(bool state) {
	bottom.set(state);
}

bool GearSubsystem::getBottom() {
	return bottom.get();
}
