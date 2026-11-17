# AGENTS.md

Embedded firmware repository for ESP32-family MCUs (mainly Seeed **XIAO ESP32-S3**).
Two toolchains: **ESP-IDF** (`idf.py`, C) and **Arduino** (`arduino-cli`, C++).
Examples live under `examples/`.

## Core principles (always apply)

- **Verify by running.** "It compiles" is not done. Make the smallest change you can
  verify on the target, and state what was / wasn't verified.
- **Respect the MCU.** Mind memory, stack, interrupt latency, and power budgets.
- **Never guess hardware.** Read `sdkconfig`, `partitions.csv`, and board init before
  assuming pins, peripherals, or enabled components.
- **Keep it layered.** One module per responsibility; dependencies point one way
  (drivers ← core ← interface ← app). No magic numbers.
- **Short ISRs.** An interrupt handler only clears the flag and signals — no `printf`,
  `malloc`, or blocking calls. Do the work in a task / the loop.
- **Thin entry points.** Keep `app_main()` / `loop()` small; delegate to named functions.
- **Match local style.** Follow neighboring files' include and naming conventions.
- **Evidence over claims.** Report the command you ran and its actual output.

## Common pitfalls to keep in mind

- ESP32-S3 USB-CDC `Serial` stalls the whole loop when no serial monitor is open.
- `arduino-cli upload` silently reuses a stale build cache — `compile --clean` first.
- BLE peripherals must re-advertise after disconnect or they vanish until reset.
- `WiFiClient` is value-semantic: stopping a saved copy RSTs the live connection.
- After flashing over USB-Serial/JTAG, boot with `--after watchdog-reset`.

## Missing development dependencies

If a build tool is missing or misconfigured (ESP-IDF + its Python env, `arduino-cli`,
or the ESP32 Arduino core), get the user's permission before installing, and confirm the
install location and requirements with them first — do **not** install silently. Record
the agreed locations in `docs/dependencies.md` so both you and the user can look them up later.

## Project skill (mandatory load)

The full workflow, playbook, and guardrails for this repo live in the project
skill at `.agents/skills/embedded-vibe-assistant/SKILL.md`. Because project-level
skills under `.agents/skills/` are protected by project trust and may **not** be
auto-mounted into the session, do the following at the start of any embedded
firmware task:

1. Check `available_skills` for `embedded-vibe-assistant`.
2. If it is **not** listed, read
   `.agents/skills/embedded-vibe-assistant/SKILL.md` directly and apply it.
3. If it **is** listed, load it normally.

Never skip the skill's operating procedure, debug playbook, or guardrails just
because it did not auto-mount.