#include "WPILib.h"

#ifndef SRC_IR_H
#define SRC_IR_H

class IR {

  DigitalInput ir;
  
public:
  IR(int channel);
  bool get();
};

#endif
