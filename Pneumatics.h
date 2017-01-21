/*
 * Pneumatics.h
 *
 *  Created on: October 25, 2016
 *      Author: Noah
 */

#include "WPILib.h"
#include "Constants.h"

#ifndef SRC_PNEUMATICS_H
#define SRC_PNEUMATICS_H

class Pneumatics {
private:
  Solenoid InSole;
  Solenoid OutSole;

public:
  Pneumatics(uint32_t InSoleChannel, uint32_t OutSoleChannel);
  void set(bool state);
  bool get();
};

#endif
