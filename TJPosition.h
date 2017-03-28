/*#include "WPILib.h"
#ifndef SRC_TJPOSITION_H_
#define SRC_TJPOSITION_H_
class TJPosition
{
	float x;
	float y;
	Encoder encoX;
	Encoder encoY;
	Ultrasonic leftProx;
	Ultrasonic rightProx;
public:
	TJPosition();
	void getLagPosition();
	void getProbablyPosition();
};
#endif
