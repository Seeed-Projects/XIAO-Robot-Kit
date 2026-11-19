# XIAO Robot Kit — firmware examples

Three self-contained examples for the **XIAO Robot Kit** (Seeed Studio, SKU 100058302), plus the
agent rules this code base is written against.

## Layout

| Path | What it is | Toolchain |
| --- | --- | --- |
| `examples/xiao_robot_kit_ble_control/` | Full-featured firmware: 45 BLE commands, speed PID, IMU-guided moves, arm gestures, CSV sample logging | ESP-IDF v6.1 |
| `examples/xiao_robot_kit_web_remote/` | Drive from a phone browser: HTTP + WebSocket page over Wi-Fi, no app, no third-party library | Arduino (ESP32 core 3.x) |
| `examples/xiao_robot_kit_ble_skill/` | The BLE firmware plus a phone agent skill: a Markdown file that turns a spoken sentence into BLE frames, a gimbal gesture and music | ESP-IDF v6.0.2+ |
| `AGENTS.md` | Standing rules for coding agents working in this repo | — |
| `.agents/skills/embedded-vibe-assistant/` | Working manual: build-chain workflow, code guardrails, eight debugging playbooks, definition of done | — |
| `.claude/skills/embedded-vibe-assistant/` | Same skill, as a short Claude Code entry point | — |

## BLE firmware (ESP-IDF)

```bash
cd examples/xiao_robot_kit_ble_control
idf.py set-target esp32s3
idf.py build
idf.py -p <serial-port> flash monitor
```

`run.bat` does build and flash on Windows; it expects `IDF_PATH` to point at ESP-IDF v6.1
(or falls back to `%USERPROFILE%\esp\v6.1\esp-idf`).

Flash layout, PSRAM mode and console are pinned in `sdkconfig.defaults` and `partitions.csv`:
16 MB flash, 8 MB octal PSRAM, USB-Serial/JTAG console. The full command set is documented in the
[example README](examples/xiao_robot_kit_ble_control/README.md).

## Web remote (Arduino)

```bash
cd examples/xiao_robot_kit_web_remote
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3_Plus
arduino-cli upload  --fqbn esp32:esp32:XIAO_ESP32S3_Plus -p <serial-port>
```

Set `WIFI_SSID` / `WIFI_PASS` in `src/app/car_config.h`, open the serial monitor at 115200, and
browse to `http://xiaorobot.local` (mDNS) or the printed IP address.

## Phone agent skill (BLE + YOYO)

`examples/xiao_robot_kit_ble_skill/` pairs the BLE firmware with `skill/honor-robotphone-base/SKILL.md`,
the file an Honor RobotPhone assistant loads to drive the car. Flash the firmware, read the name off
the label, zip the skill and import it — the steps and the first-run test list are in
[the skill guide](examples/xiao_robot_kit_ble_skill/skill/README.md).

## Working with an AI agent

`AGENTS.md` is loaded automatically by Claude Code and pi. Any other agent needs one sentence:

> Read `AGENTS.md` and `.agents/skills/embedded-vibe-assistant/SKILL.md` before touching code.

The rules that matter most: verify by running, never guess hardware, keep ISRs short, dependencies
point one way, and report the exact command and output behind every "done".