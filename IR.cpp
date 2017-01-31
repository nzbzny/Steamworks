#include "IR.h"

IR::IR(int channel) :
  ir(channel)
  {}
    
//flip the IR values
void IR::get() {
  return !ir.Get();
}
