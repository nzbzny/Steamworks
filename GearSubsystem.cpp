#include "GearSubsystem.h"

GearSubsystem::GearSubsystem(uint32_t bottomInSole, uint32_t bottomOutSole, uint32_t gearPusherInSole, uint32_t gearPusherOutSole) :
	bottomPneumatic(bottomInSole, bottomOutSole),
	gearPusher(gearPusherInSole, gearPusherOutSole)
{}

void GearSubsystem::setBottom(bool state) { //open / close the release mechanism
	bottomPneumatic.set(state);
}

bool GearSubsystem::getBottom() { //return state of the bottom
	return bottomPneumatic.get(); //TODO: may need to flip
	//true = open
	//false = closed
}

void GearSubsystem::setPusher(bool state) {
	gearPusher.set(state);
}

bool GearSubsystem::getPusher() {
	return gearPusher.get();
}
