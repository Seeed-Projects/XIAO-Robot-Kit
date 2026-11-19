# xiao_robot_kit_ble_skill — BLE firmware + a phone agent skill

The same chassis firmware as
[`xiao_robot_kit_ble_control`](../xiao_robot_kit_ble_control/), plus the piece that makes a phone
assistant drive it: **[`skill/`](skill/)**, one Markdown file that maps a spoken sentence to a BLE
command, and its import guide.

Two differences from `ble_control`:

| | `ble_control` | this example |
| --- | --- | --- |
| BLE name | fixed default, renamed with `btname` | `XIAO-Robot-Kit-` + last two bytes of the BLE address, so a flashed batch never collides |
| Companion | none | `skill/` — a YOYO skill for the Honor RobotPhone |

Everything else — command set, speed PID, IMU moves, arm gestures, dance, logging — is identical.
The full command reference is in
[`../xiao_robot_kit_ble_control/README.md`](../xiao_robot_kit_ble_control/README.md).

## Hardware

| Item | Detail |
| --- | --- |
| Board | XIAO ESP32-S3, 16 MB flash, 8 MB octal PSRAM |
| Motor driver | GC9112 (U14 / U16) |
| Motors | 2× geared DC motor with quadrature encoder |
| Servos | 2× arm servo, 50 Hz PWM |
| IMU | LSM6DS3TR-C on I2C `0x6A` |
| Console | USB Serial/JTAG |

Pins as wired on the V1.0 board:

| Motor | Body position | IN1 | IN2 |
| --- | --- | --- | --- |
| M1 / J8 | Right wheel | D1 / GPIO2 | D2 / GPIO3 |
| M2 / J9 | Left wheel | D19 / GPIO11 | D18 / GPIO12 |

| Encoder | A | B |
| --- | --- | --- |
| M1 / J8 | D3 / GPIO4 | D14 / GPIO41 |
| M2 / J9 | D17 / GPIO13 | D10 / GPIO9 |

| Servo | Body position | Connector | GPIO |
| --- | --- | --- | --- |
| SERVO1 | Right arm | J1 | D15 / GPIO42 |
| SERVO2 | Left arm | J5 | D9 / GPIO8 |

M2 is mounted mirrored; its motor polarity and encoder direction are calibrated in the firmware, so
`forward` turns both wheels forward and both counters count up.

## Build and flash

ESP-IDF **v6.0.2 or newer** recommended. v6.0.2 is the version this example was built and tested on;
`xiao_robot_kit_ble_control` runs on v6.1. If the ESP-IDF you already have is a different version,
adapt the project to it first; install v6.0.2 only if yours cannot be made to build.

```bash
source <ESP_IDF_ROOT>/export.sh
cd examples/xiao_robot_kit_ble_skill
idf.py set-target esp32s3
idf.py -p <serial-port> flash monitor
```

Target, flash size, PSRAM mode, partition table and console are pinned in `sdkconfig.defaults` and
`partitions.csv`. On Windows `run.bat` builds only; `run.bat COMx` builds, flashes and opens the
monitor.

## The BLE name

An unnamed board advertises as `XIAO-Robot-Kit-3C7A` — the prefix plus the last two bytes of its BLE
address — so ten boards out of the flashing jig already have ten different names. The name lives in
NVS, so renaming survives a reboot and a re-flash of the app partition, and one binary serves the
whole batch:

```text
btname set XIAO-Robot-Kit-01     # 1-26 chars, A-Z a-z 0-9 . _ -
btname show                      # saved name plus the MAC-derived default
btname reset                     # back to the MAC-derived default
```

Print the name on a label when you rename, or read it from the monitor log:

```text
BLE device name: XIAO-Robot-Kit-3C7A (default from MAC)
Advertising as XIAO-Robot-Kit-3C7A (general discoverable, connectable)
```

## BLE interface

| Item | Value |
| --- | --- |
| Service UUID | `a2f39c10-7e4b-4d21-8a61-0f3bc9d15001` |
| Command / status UUID | `a2f39c11-7e4b-4d21-8a61-0f3bc9d15001` |
| Frame | `M\|left\|right\|$`, `-100..100` per wheel |
| Encoding | `utf-8`, write without response |

```text
M|20|20|$      forward
M|-20|-20|$    backward
M|-20|20|$     left
M|20|-20|$     right
M|0|0|$        stop
```

A good frame is answered with `OK|M|left|right|$`, a bad one with `ERR|...|$`. The BLE text goes to
the same dispatcher as the USB shell, so every shell command — `speed`, `greeting`,
`dance_new_believer`, `cpslog`, `btname` — is reachable over Bluetooth as well. The monitor prints
each frame it receives:

```text
BLE RX (conn=1, len=9): M|40|40|$
BLE drive: left=40% right=40%
```

A disconnect or a NimBLE reset stops both motors immediately.

## The skill

[`skill/honor-robotphone-base/SKILL.md`](skill/honor-robotphone-base/SKILL.md) is the file the phone
assistant loads. It is not code: frontmatter, a device-name rule, an intent→command table, and the
timing rules that keep the car and the phone's camera gimbal in step.

- The skill never scans for the car. On the first command it asks for the last four characters of
  the label (`3C7A`) and keeps that name for the rest of the conversation.
- The car's own moves come from the BLE table above. The gimbal, the music and the natural language
  belong to the phone.
- Import, allowed tools and the first-run test list: [`skill/README.md`](skill/README.md).

## Layout

```text
main/main.c             USB shell and application entry
main/ble_motor.*        NimBLE GATT service, BLE frame parser, NVS device name
main/motor_control.*    GC9112 PWM and direction
main/encoder.*          Quadrature encoders and polarity correction
main/speed_control.*    Per-wheel PID with direction-split feedforward
main/chassis_action.*   IMU-guided moves, reverse park
main/arm_action.*       Arm presets and gesture interpolation
main/dance.c            Beat timelines for the two dances
main/cps_log.*          RAM sample buffer and SPIFFS log partition
main/vofa_telemetry.*   J10 UART0 FireWater telemetry
skill/                  YOYO skill source, import guide, upload package
```

## Safety

- Test with the wheels off the ground and a hand on the power switch the first time.
- Drive the motors from a separate motor supply through the GC9112 — never from the 3.3 V pin.
- This build has no global auto-stop: every non-zero `speed` keeps running until `stop` arrives or
  the BLE link drops.