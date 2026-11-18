#pragma once
#include "Arduino.h"
namespace sim { extern int servo_us[2]; extern int servo_n; }
class Servo {
  int idx = -1;
 public:
  void attach(int) { idx = sim::servo_n++; }
  void writeMicroseconds(int us) { if (idx >= 0) sim::servo_us[idx] = us; }
};
