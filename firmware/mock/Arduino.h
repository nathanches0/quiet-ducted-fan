// Minimal host-side mock of the Arduino API for testing rig_logger.ino with a simulated rig.
#pragma once
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <string>
#include <deque>
#include <functional>
#include <cmath>

typedef uint8_t byte;
class __FlashStringHelper;
#define F(s) (reinterpret_cast<const __FlashStringHelper *>(s))
enum { LOW = 0, HIGH = 1, INPUT = 0, OUTPUT = 1, INPUT_PULLUP = 2, FALLING = 2 };
enum { A0 = 14, A1 = 15 };

namespace sim {
  extern unsigned long now_ms;
  extern int stop_pin;                       // 1 = released, 0 = pressed
  extern std::function<int(uint8_t)> analog;
  extern void (*isr[2])();
  extern std::string out;
  extern std::deque<char> in;
}
inline unsigned long millis() { return sim::now_ms; }
inline void pinMode(uint8_t, uint8_t) {}
inline int digitalRead(uint8_t pin) { return pin == 7 ? sim::stop_pin : HIGH; }
inline int analogRead(uint8_t pin) { return sim::analog(pin); }
inline int digitalPinToInterrupt(uint8_t pin) { return pin == 2 ? 0 : 1; }
inline void attachInterrupt(int n, void (*f)(), int) { sim::isr[n] = f; }
inline void noInterrupts() {}
inline void interrupts() {}

struct MockSerial {
  void begin(long) {}
  int available() { return (int)sim::in.size(); }
  int read() { if (sim::in.empty()) return -1; char c = sim::in.front(); sim::in.pop_front(); return c; }
  void print(const __FlashStringHelper *s) { sim::out += reinterpret_cast<const char *>(s); }
  void print(const char *s) { sim::out += s; }
  void print(char c) { sim::out += c; }
  void print(bool b) { sim::out += b ? "1" : "0"; }
  void print(int v) { sim::out += std::to_string(v); }
  void print(unsigned long v) { sim::out += std::to_string(v); }
  void print(long v) { sim::out += std::to_string(v); }
  void print(double v, int d = 2) { char b[64]; snprintf(b, sizeof b, "%.*f", d, v); sim::out += b; }
  template <typename T> void println(T v) { print(v); sim::out += "\n"; }
  void println(double v, int d) { print(v, d); sim::out += "\n"; }
  void println() { sim::out += "\n"; }
};
extern MockSerial Serial;
