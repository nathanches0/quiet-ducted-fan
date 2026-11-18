#pragma once
#include "Arduino.h"
namespace sim { extern bool hx_ready(); extern long hx_read(); }
class HX711 {
 public:
  void begin(int, int) {}
  bool is_ready() { return sim::hx_ready(); }
  long read() { return sim::hx_read(); }
};
