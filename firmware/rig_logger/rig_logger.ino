/*
  Quiet Ducted Fan - bench rig firmware (Arduino Nano)

  Drives two ESCs (upper and lower rotor) and streams thrust, rpm, current,
  voltage and power to the laptop as CSV at 20 rows per second.
  Use it with scripts/rig_log.py, which saves each run to results/.

  Wiring (see the console's Wiring tab):
    D9  -> ESC 1 signal (upper rotor)     D10 -> ESC 2 signal (lower rotor)
    D2  <- hall sensor, upper motor       D3  <- hall sensor, lower motor
    D5  <- HX711 DT                       D6  -> HX711 SCK
    D7  <- stop button to GND
    A0  <- ACS758 current sensor OUT      A1  <- battery voltage divider (10k / 3.3k)
    Every ESC signal ground, sensor ground and the divider go to Nano GND.

  Library: "HX711 Arduino Library" by Bogdan Necula (Library Manager).

  SAFETY - the real off switch is the arming plug in the battery lead.
  This code adds three software layers on top:
    1. The stop button (D7) or the "s" command cuts both throttles at once
       and latches. Nothing restarts until you send "r" with the button released.
    2. Watchdog: if the laptop goes quiet for 1.5 s while armed, it stops.
       rig_log.py sends a heartbeat for you. In the Serial Monitor, "w 0"
       turns the watchdog off (only while disarmed).
    3. Throttle changes are ramped (20 % per second), so nothing jumps.

  Commands (type a line, press Enter):
    a          arm (throttle must be 0 and stop not latched)
    s          STOP: throttles to zero, disarm
    r          reset after a stop (button must be released)
    t 40       both rotors to 40 %
    u 40       upper rotor only        l 40   lower rotor only
    k 3        trim: lower runs 3 % above the upper (use to balance torque)
    f 8.5      hold 8.5 N of thrust (adjusts both throttles together)
    m 80       limit throttle to 80 %
    z          zero the load cell and current sensor (motors stopped)
    c 500      calibrate: place a 500 g mass along the thrust axis, then send
    w 0 / w 1  watchdog off / on
    p          heartbeat (any line also counts)
    ?          print status and help
*/

#include <Servo.h>
#include <HX711.h>
#include <EEPROM.h>

// ---------------- pins ----------------
const uint8_t PIN_ESC_UPPER = 9;
const uint8_t PIN_ESC_LOWER = 10;
const uint8_t PIN_HALL_UPPER = 2;
const uint8_t PIN_HALL_LOWER = 3;
const uint8_t PIN_HX_DT = 5;
const uint8_t PIN_HX_SCK = 6;
const uint8_t PIN_STOP = 7;
const uint8_t PIN_CURRENT = A0;
const uint8_t PIN_VOLTAGE = A1;

// ---------------- settings ----------------
const int US_MIN = 1000;                 // ESC pulse for stop
const int US_MAX = 2000;                 // ESC pulse for full throttle
const float RAMP_PCT_PER_S = 20.0;       // fastest throttle change
const unsigned long WATCHDOG_MS = 1500;  // stop if the laptop goes quiet this long
const unsigned long LOG_MS = 50;         // one CSV row every 50 ms (20 Hz)
const unsigned long RPM_WINDOW_MS = 250; // rpm averaging window
const unsigned long ESC_INIT_MS = 3000;  // hold minimum throttle at power-up so ESCs arm
const uint8_t MAGNETS_PER_REV = 2;       // two magnets per motor bell
const float ADC_REF_V = 5.00;            // measure the Nano's 5V pin with a multimeter and put it here
const float DIVIDER_RATIO = (10.0 + 3.3) / 3.3;
const float CURRENT_MV_PER_A = 40.0;     // ACS758LCB-050B: 40 mV/A. ACS758LCB-050U: 60 mV/A
const float HOLD_GAIN = 3.0;             // thrust hold: % throttle per newton-second of error
const float G = 9.80665;

// ---------------- state ----------------
Servo escUpper, escLower;
HX711 scale;

volatile uint16_t pulsesUpper = 0, pulsesLower = 0;

bool armed = false;
bool latched = false;
bool watchdogOn = true;
float maxPct = 100.0;
float trimPct = 0.0;            // lower minus upper
float targetUpper = 0, targetLower = 0;   // requested %
float outUpper = 0, outLower = 0;         // ramped %, sent to ESCs
float holdN = -1;               // thrust hold target, -1 = off
float holdBase = 0;             // throttle the hold loop is using

long scaleOffset = 0;           // raw HX711 counts at zero load
float countsPerGram = 420.0;    // replaced by "c" calibration, saved in EEPROM
long lastRaw = 0;
float thrustN = 0;

float currentZeroAdc = 512;
float rpmUpper = 0, rpmLower = 0;

unsigned long lastRx = 0, lastLog = 0, lastRpm = 0, lastLoop = 0, bootMs = 0;
char line[32];
uint8_t lineLen = 0;

const uint16_t EE_MAGIC = 0xFA11;

// ---------------- interrupts ----------------
void hallUpper() { pulsesUpper++; }
void hallLower() { pulsesLower++; }

// ---------------- helpers ----------------
float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

int pctToUs(float pct) { return US_MIN + (int)(clampf(pct, 0, 100) / 100.0 * (US_MAX - US_MIN) + 0.5); }

void msg(const __FlashStringHelper *m) { Serial.print(F("# ")); Serial.println(m); }

void stopAll(const __FlashStringHelper *why) {
  targetUpper = targetLower = outUpper = outLower = 0;
  holdN = -1;
  holdBase = 0;
  armed = false;
  escUpper.writeMicroseconds(US_MIN);
  escLower.writeMicroseconds(US_MIN);
  Serial.print(F("# STOP: "));
  Serial.println(why);
}

void setBoth(float pct) {
  pct = clampf(pct, 0, maxPct);
  targetUpper = pct;
  targetLower = clampf(pct + trimPct, 0, maxPct);
}

float readAdcAvg(uint8_t pin, uint8_t n) {
  long sum = 0;
  for (uint8_t i = 0; i < n; i++) sum += analogRead(pin);
  return (float)sum / n;
}

long readRawAvg(uint8_t n) {
  long sum = 0;
  uint8_t got = 0;
  unsigned long t0 = millis();
  while (got < n && millis() - t0 < 3000) {
    if (scale.is_ready()) { sum += scale.read(); got++; }
  }
  return got ? sum / got : lastRaw;
}

void saveCal() {
  EEPROM.put(0, EE_MAGIC);
  EEPROM.put(2, countsPerGram);
}

void loadCal() {
  uint16_t magic;
  EEPROM.get(0, magic);
  if (magic == EE_MAGIC) {
    float c;
    EEPROM.get(2, c);
    if (c != 0 && c == c) countsPerGram = c;   // c == c rejects NaN
  }
}

void printHeader() {
  Serial.println(F("ms,armed,upper_pct,lower_pct,thrust_N,rpm_upper,rpm_lower,current_A,voltage_V,power_W,hold_N"));
}

void printStatus() {
  Serial.print(F("# armed=")); Serial.print(armed);
  Serial.print(F(" latched=")); Serial.print(latched);
  Serial.print(F(" watchdog=")); Serial.print(watchdogOn);
  Serial.print(F(" max=")); Serial.print(maxPct, 0);
  Serial.print(F("% trim=")); Serial.print(trimPct, 1);
  Serial.print(F("% countsPerGram=")); Serial.println(countsPerGram, 2);
  msg(F("commands: a arm | s stop | r reset | t/u/l <pct> | k <trim> | f <N> | m <max> | z zero | c <grams> | w 0/1 | ?"));
}

// ---------------- commands ----------------
void handleCommand(char *cmd) {
  while (*cmd == ' ') cmd++;
  char c = cmd[0];
  float v = atof(cmd + 1);
  bool hasArg = false;
  for (char *p = cmd + 1; *p; p++) if ((*p >= '0' && *p <= '9') || *p == '-' || *p == '.') { hasArg = true; break; }

  switch (c) {
    case 'p': break;                                   // heartbeat
    case '?': printStatus(); break;
    case 's': stopAll(F("command")); break;
    case 'a':
      if (latched) { msg(F("stop is latched: release the button and send r")); break; }
      if (millis() - bootMs < ESC_INIT_MS) { msg(F("ESCs still starting, try again in a moment")); break; }
      if (targetUpper > 0 || targetLower > 0) { msg(F("set throttle to 0 before arming")); break; }
      armed = true; msg(F("ARMED"));
      break;
    case 'r':
      if (digitalRead(PIN_STOP) == LOW) { msg(F("stop button still pressed")); break; }
      latched = false; msg(F("stop reset (send a to arm)"));
      break;
    case 't': case 'u': case 'l':
      if (!hasArg) { msg(F("needs a number, e.g. t 40")); break; }
      if (!armed) { msg(F("not armed")); break; }
      holdN = -1;
      if (c == 't') setBoth(v);
      else if (c == 'u') targetUpper = clampf(v, 0, maxPct);
      else targetLower = clampf(v, 0, maxPct);
      break;
    case 'k':
      if (!hasArg) { msg(F("needs a number, e.g. k 3")); break; }
      trimPct = clampf(v, -20, 20);
      if (holdN < 0) setBoth(targetUpper);
      break;
    case 'f':
      if (!armed) { msg(F("not armed")); break; }
      if (!hasArg || v <= 0) { holdN = -1; msg(F("thrust hold off")); break; }
      holdN = v; holdBase = targetUpper; msg(F("thrust hold on"));
      break;
    case 'm':
      if (!hasArg) { msg(F("needs a number, e.g. m 80")); break; }
      maxPct = clampf(v, 0, 100);
      targetUpper = clampf(targetUpper, 0, maxPct);
      targetLower = clampf(targetLower, 0, maxPct);
      break;
    case 'z':
      if (outUpper > 0 || outLower > 0) { msg(F("stop the motors before zeroing")); break; }
      scaleOffset = readRawAvg(16);
      currentZeroAdc = readAdcAvg(PIN_CURRENT, 64);
      msg(F("zeroed load cell and current sensor"));
      break;
    case 'c':
      if (!hasArg || v <= 0) { msg(F("send the mass in grams, e.g. c 500")); break; }
      {
        long raw = readRawAvg(16);
        float cpg = (raw - scaleOffset) / v;
        if (cpg > -1 && cpg < 1) { msg(F("reading barely changed: check the mass and wiring")); break; }
        countsPerGram = cpg;
        saveCal();
        msg(F("calibrated and saved"));
      }
      break;
    case 'w':
      if (!hasArg) { msg(F("send w 0 (off) or w 1 (on)")); break; }
      if (v == 0 && armed) { msg(F("disarm before turning the watchdog off")); break; }
      watchdogOn = (v != 0);
      msg(watchdogOn ? F("watchdog on") : F("watchdog OFF"));
      break;
    default:
      msg(F("unknown command, send ? for help"));
  }
}

void readSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    lastRx = millis();
    if (ch == '\n' || ch == '\r') {
      if (lineLen) { line[lineLen] = 0; handleCommand(line); lineLen = 0; }
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = (ch >= 'A' && ch <= 'Z') ? ch + 32 : ch;
    }
  }
}

// ---------------- setup / loop ----------------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_STOP, INPUT_PULLUP);
  pinMode(PIN_HALL_UPPER, INPUT_PULLUP);
  pinMode(PIN_HALL_LOWER, INPUT_PULLUP);

  escUpper.attach(PIN_ESC_UPPER);
  escLower.attach(PIN_ESC_LOWER);
  escUpper.writeMicroseconds(US_MIN);
  escLower.writeMicroseconds(US_MIN);

  attachInterrupt(digitalPinToInterrupt(PIN_HALL_UPPER), hallUpper, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL_LOWER), hallLower, FALLING);

  scale.begin(PIN_HX_DT, PIN_HX_SCK);
  loadCal();
  scaleOffset = readRawAvg(16);
  currentZeroAdc = readAdcAvg(PIN_CURRENT, 64);

  bootMs = lastRx = lastLog = lastRpm = lastLoop = millis();
  msg(F("Quiet Ducted Fan rig ready. Disarmed. Send ? for help."));
  printHeader();
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastLoop) / 1000.0;
  lastLoop = now;

  readSerial();

  // --- safety ---
  if (digitalRead(PIN_STOP) == LOW && !latched) {
    latched = true;
    stopAll(F("button"));
  }
  if (armed && watchdogOn && now - lastRx > WATCHDOG_MS) stopAll(F("watchdog (laptop went quiet)"));

  // --- sensors ---
  if (scale.is_ready()) {
    lastRaw = scale.read();
    thrustN = (lastRaw - scaleOffset) / countsPerGram / 1000.0 * G;
  }
  if (now - lastRpm >= RPM_WINDOW_MS) {
    noInterrupts();
    uint16_t pu = pulsesUpper, pl = pulsesLower;
    pulsesUpper = pulsesLower = 0;
    interrupts();
    float win = (now - lastRpm) / 60000.0;   // minutes
    rpmUpper = pu / (float)MAGNETS_PER_REV / win;
    rpmLower = pl / (float)MAGNETS_PER_REV / win;
    lastRpm = now;
  }
  float amps = (analogRead(PIN_CURRENT) - currentZeroAdc) * ADC_REF_V / 1023.0 * 1000.0 / CURRENT_MV_PER_A;
  float volts = analogRead(PIN_VOLTAGE) * ADC_REF_V / 1023.0 * DIVIDER_RATIO;

  // --- thrust hold ---
  if (armed && holdN > 0) {
    holdBase = clampf(holdBase + HOLD_GAIN * (holdN - thrustN) * dt, 0, maxPct);
    setBoth(holdBase);
    // a reversed or uncalibrated load cell would make the hold loop run away: catch it
    if (outUpper > 30 && thrustN < 0.2) {
      holdN = -1;
      setBoth(0);
      msg(F("thrust hold off: no thrust reading at 30 % (check load cell direction and calibration)"));
    }
  }

  // --- ramp and output ---
  if (!armed) { targetUpper = targetLower = 0; outUpper = outLower = 0; }
  float step = RAMP_PCT_PER_S * dt;
  outUpper += clampf(targetUpper - outUpper, -step, step);
  outLower += clampf(targetLower - outLower, -step, step);
  escUpper.writeMicroseconds(armed ? pctToUs(outUpper) : US_MIN);
  escLower.writeMicroseconds(armed ? pctToUs(outLower) : US_MIN);

  // --- log ---
  if (now - lastLog >= LOG_MS) {
    lastLog = now;
    Serial.print(now); Serial.print(',');
    Serial.print(armed); Serial.print(',');
    Serial.print(outUpper, 1); Serial.print(',');
    Serial.print(outLower, 1); Serial.print(',');
    Serial.print(thrustN, 3); Serial.print(',');
    Serial.print(rpmUpper, 0); Serial.print(',');
    Serial.print(rpmLower, 0); Serial.print(',');
    Serial.print(amps, 2); Serial.print(',');
    Serial.print(volts, 2); Serial.print(',');
    Serial.print(amps * volts, 1); Serial.print(',');
    Serial.println(holdN > 0 ? holdN : 0, 2);
  }
}
