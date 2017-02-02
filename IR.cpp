#include "IR.h"

IR::IR(int channel) :
  ir(channel)
  {}
    
bool IR::get() { //flip the IR values
  return !ir.Get();
}
