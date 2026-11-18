# Bench rig firmware

`rig_logger/rig_logger.ino` runs on the Arduino Nano. It drives both ESCs, reads thrust, rpm, current and voltage, and streams them to the laptop at 20 rows per second. `scripts/rig_log.py` on the laptop saves each run to `results/`.

Wiring is in the [Quiet Fan Console](https://claude.ai/artifact/XcCDvP2ELBb7qd5he5Hazy) (Wiring tab) and at the top of the `.ino` file.

## Setup

1. Install the Arduino IDE. In Library Manager, install **HX711 Arduino Library** by Bogdan Necula.
2. Open `rig_logger/rig_logger.ino`, choose **Arduino Nano** (try "ATmega328P (Old Bootloader)" if upload fails) and upload.
3. Measure the Nano's 5V pin with a multimeter and put the value in `ADC_REF_V`. Check your current sensor's model: `CURRENT_MV_PER_A` is 40 for ACS758LCB-050B, 60 for -050U.
4. On the laptop: `pip install pyserial`.

## First power-up (no rotors fitted)

Do these with the **rotors off the motors** and the arming plug out until step 4.

1. `python scripts/rig_log.py --port <port>` and check that rows are arriving (use `--list` to find the port).
2. With nothing on the load cell, send `z`. Hang a known mass on the thrust axis (e.g. 500 g) and send `c 500`. The calibration is saved on the Nano.
3. Press the stop button: you should see `# STOP: button`. Send `r` to reset.
4. Plug the arming plug in. Send `a`, then `t 10`: both bare motors should spin slowly, the lower one in reverse. Send `s`.
5. Unplug the USB cable while a motor spins at `t 10`: it must stop within 1.5 s (watchdog).

Only fit rotors once all five work. Balance each rotor and check the tip gap first.

## Commands

| Command | What it does |
|---|---|
| `a` | arm (throttle must be 0) |
| `s` | **stop**: both throttles to zero, disarm |
| `r` | reset after a stop (button released) |
| `t 40` | both rotors to 40 % |
| `u 40` / `l 40` | upper / lower rotor only |
| `k 3` | trim: lower runs 3 % above upper (balance the torque) |
| `f 8.5` | hold 8.5 N of thrust |
| `m 80` | limit throttle to 80 % |
| `z` | zero load cell and current sensor (motors stopped) |
| `c 500` | calibrate with a 500 g mass |
| `w 0` / `w 1` | watchdog off / on (off only while disarmed) |
| `?` | status and help |

## Safety layers

The **arming plug is the real off switch.** The firmware adds three more layers:

- **Stop latch:** the stop button or `s` cuts both throttles at once and latches until `r`.
- **Watchdog:** if the laptop goes quiet for 1.5 s while armed, it stops. The logger sends the heartbeat for you.
- **Ramp and limits:** throttle moves at most 20 % per second and never above `m`. If the thrust hold sees no thrust at 30 % throttle (load cell reversed or uncalibrated), it switches itself off instead of running up to full throttle.

## Logging a test

Equal-thrust sweep (the test plan's method):

```bash
python scripts/rig_log.py --port COM5 --label 15-7-23 --gap 1.5 --sweep 4,8,12
```

This arms the rig, holds each thrust until it's steady, records 10 s, then stops. It saves the raw data and a summary (mean thrust, power, N/kW, rpm per point). Record the UMIK-1 at the same time and note its file name in the CSV header.

Try it with no hardware: `python scripts/rig_log.py --sim --label test --sweep 4,8`

## Tests

`test_rig.cpp` compiles the firmware on a normal computer against a mock Arduino (`mock/`) and runs it against a simulated rig. It covers arming, ramping, rpm, calibration, thrust hold, the watchdog, the stop latch, command guards and the reversed-load-cell failsafe.

```bash
cd firmware && g++ -std=c++17 -I mock -o test_rig test_rig.cpp && ./test_rig
```

The simulated rig follows the design model. Real numbers come from the hardware.
