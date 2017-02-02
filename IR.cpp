#include "IR.h"

IR::IR(int channel) :
  ir(channel)
  {}
    
//flip the IR values
bool IR::get() {
  return !ir.Get();
}
