# Development dependencies

Two toolchains, one per example. Paths below are Windows defaults; on Linux/macOS
replace `%USERPROFILE%` with `~`.

When any of these is missing or misconfigured, get the user's permission before
installing, and agree the install location with them first — do **not** install
silently. Point them at the location below and update this file if anything changes.

## 1. ESP-IDF v6.1 — BLE example (`examples/xiao_robot_kit_ble_control/`)

| Requirement | Default location (Windows) |
| --- | --- |
| ESP-IDF SDK v6.1 | `%USERPROFILE%\esp\v6.1\esp-idf` |
| Python virtualenv | `%USERPROFILE%\.espressif\python_env\idf6.1_py3.11_env` |

- `run.bat` falls back to these paths via `IDF_PATH` / `IDF_PYTHON_ENV_PATH`, so
  either install there or set those environment variables.
- Install with the ESP-IDF installer (`idf.py setup` / the Windows offline installer).
- Manual command: `cd examples/xiao_robot_kit_ble_control && idf.py set-target esp32s3 && idf.py build`.

## 2. Arduino CLI + ESP32 core 3.x — web-remote example (`examples/xiao_robot_kit_web_remote/`)

| Requirement | Notes |
| --- | --- |
| `arduino-cli` | On `PATH` |
| ESP32 Arduino core 3.x | Board `esp32:esp32:XIAO_ESP32S3_Plus` |

Install (once) — standard `arduino-cli` steps, not pinned in any repo file (verify the index URL if corp proxy/mirror differs):

```bash
arduino-cli config init
arduino-cli config add board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32
```

Manual command:

```bash
arduino-cli compile --clean --fqbn esp32:esp32:XIAO_ESP32S3_Plus examples/xiao_robot_kit_web_remote
```