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

## Missing development dependencies

If a build tool is missing or misconfigured (ESP-IDF + its Python env, `arduino-cli`,
or the ESP32 Arduino core), get the user's permission before installing, and confirm the
install location and requirements with them first — do **not** install silently. Record
the agreed locations in `docs/dependencies.md` so both you and the user can look them up later.

## Project skill (mandatory load)

This repo's operating procedure, field-tested debug playbook, and code-quality
guardrails for embedded firmware live in one **project-level skill**:

```
.agents/skills/embedded-vibe-assistant/
  SKILL.md                          # workflow + guardrails (read this first)
  references/field-tested-rules.md  # full field-tested rule catalog
```

Load it on your **first session** in this repository (and again if it changes),
before you touch any embedded firmware task. Each AI IDE mounts skills its own
way — use **your IDE's native mechanism**:

1. **If your IDE supports skills / rules / instructions** (e.g. Claude Code skills,
   Cursor rules, Windsurf, Cline rules, Codex AGENTS, Gemini CLI, Copilot
   instructions, or your local equivalent): mount/import this project skill through
   that mechanism. If it was not auto-discovered, point your editor at the skill
   directory above.
2. **If your IDE has no skill mechanism:** read `SKILL.md` (and its `references/`)
   above directly and treat every rule in it as a hard instruction for this repo.

Never skip the skill's operating procedure, debug playbook, or guardrails just
because it did not auto-load in your editor.