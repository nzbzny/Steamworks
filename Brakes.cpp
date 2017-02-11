#include "Brakes.h"

Brakes::Brakes(int brakeInSole, int brakeOutSole) :
	brake(brakeInSole, brakeOutSole)
{}

void Brakes::set(bool state) {
	brake.set(state);
}

bool Brakes::get() {
	return brake.get(); //TODO: may need to flip
}
