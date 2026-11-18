// Host test: compiles rig_logger.ino against mocks and drives it with a simulated rig.
#include "Arduino.h"
#include <random>
#include <vector>
#include <sstream>
#include <iostream>

namespace sim {
unsigned long now_ms = 0;
int stop_pin = HIGH;
std::function<int(uint8_t)> analog;
void (*isr[2])() = {nullptr, nullptr};
std::string out;
std::deque<char> in;
int servo_us[2] = {0, 0};
int servo_n = 0;
uint8_t eeprom[64] = {0};
// plant
double rpmU = 0, rpmL = 0, massG = 0, accU = 0, accL = 0, volts = 16.0, amps = 0;
const double TRUE_CPG = 437.0;
unsigned long lastHx = 0;
std::mt19937 rng(1);
unsigned long lastPoll = (unsigned long)-1;
bool hx_ready() {
  // a busy-wait (readRawAvg) polls many times in the same millisecond: let time pass
  if (now_ms == lastPoll) now_ms++;
  lastPoll = now_ms;
  return now_ms - lastHx >= 13;
}
double thrustN() { return 16.3 * (pow(rpmU / 15300, 2) + pow(rpmL / 15300, 2)) / 2; }
long hx_read() {
  lastHx = now_ms;
  std::normal_distribution<double> n(0, 150);
  double grams = thrustN() / 9.80665 * 1000 + massG;
  return (long)(8000000 + TRUE_CPG * grams + n(rng));
}
}  // namespace sim
#include "rig_logger/rig_logger.ino"
MockSerial Serial;
MockEEPROM EEPROM;

using namespace sim;
int fails = 0;
void check(bool ok, const std::string &what) {
  std::cout << (ok ? "  PASS  " : "  FAIL  ") << what << "\n";
  if (!ok) fails++;
}
void send(const std::string &s) { for (char c : s) in.push_back(c); in.push_back('\n'); }
void step(unsigned long ms, bool heartbeat = true) {
  for (unsigned long i = 0; i < ms; i++) {
    now_ms++;
    double dt = 0.001;
    auto pct = [](int us) { return us <= 1000 ? 0.0 : (us - 1000) / 10.0; };
    double tU = 16000 * pct(servo_us[0]) / 100, tL = 16000 * pct(servo_us[1]) / 100;
    rpmU += (tU - rpmU) * dt / 0.15;
    rpmL += (tL - rpmL) * dt / 0.15;
    accU += rpmU / 60 * 2 * dt; accL += rpmL / 60 * 2 * dt;
    while (accU >= 1) { accU -= 1; if (isr[0]) isr[0](); }
    while (accL >= 1) { accL -= 1; if (isr[1]) isr[1](); }
    double P = 316 * (pow(rpmU / 15300, 3) + pow(rpmL / 15300, 3)) / 2;
    volts = 16.0 - 0.02 * amps; amps = P / volts;
    if (heartbeat && now_ms % 500 == 0) send("p");
    loop();
  }
}
struct Row { double ms, armed, up, lo, thrust, rpmU, rpmL, amps, volts, watts, hold; };
Row lastRow() {
  std::istringstream ss(out); std::string ln; Row r{};
  while (std::getline(ss, ln)) {
    if (ln.empty() || !isdigit((unsigned char)ln[0])) continue;
    std::vector<double> v; std::stringstream ls(ln); std::string f;
    while (std::getline(ls, f, ',')) v.push_back(atof(f.c_str()));
    if (v.size() == 11) r = {v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8], v[9], v[10]};
  }
  return r;
}
bool saw(const std::string &s) { return out.find(s) != std::string::npos; }

int main() {
  analog = [](uint8_t pin) {
    if (pin == A0) return (int)lround(512 + amps * 0.040 / 5.0 * 1023);
    return (int)lround(volts / ((10.0 + 3.3) / 3.3) * 1023 / 5.0);
  };
  setup();
  check(saw("ms,armed,upper_pct"), "prints CSV header at boot");

  std::cout << "Arming\n";
  send("a"); step(100);
  check(saw("ESCs still starting"), "refuses to arm during ESC start-up");
  step(3000);
  send("t 40"); step(50);
  check(saw("not armed"), "refuses throttle while disarmed");
  send("a"); step(50);
  check(armed, "arms after start-up");

  std::cout << "Ramp\n";
  send("t 50"); step(1000);
  Row r = lastRow();
  check(r.up > 15 && r.up < 25, "throttle ramps (~20 % after 1 s, got " + std::to_string(r.up) + ")");
  step(3000); r = lastRow();
  check(fabs(r.up - 50) < 0.5 && fabs(r.lo - 50) < 0.5, "reaches 50 % on both rotors");
  check(fabs(r.rpmU - 8000) < 400, "rpm from hall sensors ~8000 (got " + std::to_string(r.rpmU) + ")");
  check(r.amps > 1 && r.volts > 15, "current and voltage read (" + std::to_string(r.amps) + " A, " + std::to_string(r.volts) + " V)");

  std::cout << "Calibration (motors stopped)\n";
  send("s"); step(2000);
  send("z"); step(50);
  massG = 500; send("c 500"); step(50);
  check(fabs(countsPerGram - TRUE_CPG) < 5, "calibration ~437 counts/g (got " + std::to_string(countsPerGram) + ")");
  massG = 0; step(200);
  r = lastRow();
  check(fabs(r.thrust) < 0.05, "reads ~0 N with nothing on it (" + std::to_string(r.thrust) + ")");

  std::cout << "Thrust reading\n";
  send("a"); step(50); send("t 80"); step(6000);
  r = lastRow();
  double expect = 16.3 * pow(12800.0 / 15300, 2);
  check(fabs(r.thrust - expect) < 0.3, "thrust matches simulated rig (" + std::to_string(r.thrust) + " N vs " + std::to_string(expect) + ")");

  std::cout << "Thrust hold\n";
  send("f 8"); step(15000);
  r = lastRow();
  check(fabs(r.thrust - 8.0) < 0.3, "holds 8 N (got " + std::to_string(r.thrust) + " N at " + std::to_string(r.up) + " %)");

  std::cout << "Watchdog\n";
  out.clear(); step(2000, false);
  check(saw("STOP: watchdog") && !armed, "stops when the laptop goes quiet");

  std::cout << "Stop button\n";
  step(500); send("a"); step(50); send("t 30"); step(2000);
  out.clear(); stop_pin = LOW; step(20);
  check(saw("STOP: button") && !armed && servo_us[0] == 1000, "button cuts throttle at once");
  send("a"); step(50); check(!armed && saw("stop is latched"), "can't re-arm while latched");
  send("r"); step(50); check(saw("still pressed"), "can't reset while button held");
  stop_pin = HIGH; send("r"); step(50); send("a"); step(50);
  check(armed, "re-arms after release + reset");

  std::cout << "Command guards\n";
  out.clear(); send("w"); step(50);
  check(watchdogOn && saw("send w 0"), "bare 'w' does not disable the watchdog");
  send("w 0"); step(50); check(watchdogOn, "can't disable watchdog while armed");
  send("t 150"); step(8000); r = lastRow();
  check(r.up <= 100.0, "throttle clamped to 100 %");
  send("m 60"); step(4000); r = lastRow();
  check(r.up <= 60.5, "max limit pulls throttle down to 60 %");

  std::cout << "Reversed load cell safety\n";
  send("s"); step(2000); countsPerGram = -countsPerGram;
  send("m 100"); send("a"); step(50); send("f 8"); step(8000);
  check(saw("thrust hold off: no thrust reading"), "hold loop gives up instead of running to full throttle");
  r = lastRow(); check(r.up < 35, "throttle came back down (" + std::to_string(r.up) + " %)");

  std::cout << (fails ? "\nSOME TESTS FAILED\n" : "\nALL TESTS PASSED\n");
  return fails ? 1 : 0;
}
