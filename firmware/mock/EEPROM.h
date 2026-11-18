#pragma once
#include "Arduino.h"
namespace sim { extern uint8_t eeprom[64]; }
struct MockEEPROM {
  template <typename T> void put(int a, const T &v) { memcpy(sim::eeprom + a, &v, sizeof(T)); }
  template <typename T> void get(int a, T &v) { memcpy(&v, sim::eeprom + a, sizeof(T)); }
};
extern MockEEPROM EEPROM;
