# Phone skill — import and first run

`honor-robotphone-base/SKILL.md` is the skill itself: a Markdown file that tells the phone assistant
which chassis commands exist, when to send them, and how to run the phone's own camera gimbal and
music player alongside them. The chassis side stays on the ESP32 — the skill only sends BLE frames.

`README.zh-CN.md` is the author's original Chinese guide; this file is the same content in English.

## Device name and connection

Every car runs the same firmware. An unnamed board advertises as `XIAO-Robot-Kit-` plus the last two
bytes of its BLE address (for example `XIAO-Robot-Kit-3C7A`); the label on the body carries that
name, and so does the phone's Bluetooth list.

The skill does **not** scan. On the first command it asks for the last four characters
(`XIAO-Robot-Kit-` is implied), builds the full name and remembers it — every later command reuses
that `device_name`. It only asks again when you say "another car" or give a new suffix.

Both UUIDs are identical on every kit:

```text
service_uuid="a2f39c10-7e4b-4d21-8a61-0f3bc9d15001"
characteristic_uuid="a2f39c11-7e4b-4d21-8a61-0f3bc9d15001"
```

So renaming a car never means editing `SKILL.md`. `btname set XIAO-Robot-Kit-01` gives it a nicer
number and a matching label; leave it alone and the MAC-derived name works as well.

> The `z1`–`w3` firmware images use different UUIDs and `XIAO-MOTOR-*` names. This skill cannot
> drive those boards.

## Linked actions

`greeting`, `emotion_happy`, `emotion_scared`, `emotion_angry`, `reverse_park` and
`dance_new_believer` are combined moves: the skill opens the gimbal, then calls the gimbal API and
the BLE command in the same turn. Only **reverse park** and **dance** also play music:

- reverse park — a local track of about eight seconds; the gimbal looks back over the shoulder in
  five four-beat bars, the way a driver checks the mirror while reversing;
- dance — `New_Believer35`, with the gimbal choreographed in nineteen four-beat bars.

`greeting` and the three emotions use one atomic gimbal gesture and no music, so the phone's motion
lasts about as long as the car's.

Both music files end with several minutes of silence on purpose. The phone agent only stays alive
for about ten seconds after a request, so it cannot come back later to stop the player: the audible
part ends, the file keeps playing silence, and nothing has to be closed. Do not trim or replace
those tails.

Arm presets (`wave_hand`, `handshake`, `open_arms`, `raise_both_arms`, `close_arms`) have no gimbal
or music attached. The open-loop frame `M|left|right|$` and the old `greet` / `dance takehand`
names are not used by the current presets.

## Build the upload package

The platform's *Import Skill* wants a ZIP whose root contains `SKILL.md`:

```bash
cd skill/honor-robotphone-base
zip -j ../honor-robotphone-base-skill.zip SKILL.md
```

## Import on the platform

1. Honor developer platform → **Management Center → Ecosystem Services → Agent Services**.
2. In **My Skills**, create a skill and choose **Import Skill**; upload the ZIP.
3. In the skill's basic information, fill the allowed-tools field with
   `basicGimbal,operateGimbal,generateGimbal,playMusic,operateMusic,ble_send_command,skill_exit`.
4. Publish it privately, sign in to the assistant on the RobotPhone with the same account, then
   install it from **Skill Store → My Skills → Private Skills**.

> The `allowed-tools` line inside `SKILL.md` is not enough. Without the same list saved in the
> platform's basic information the skill may fail to load on the phone. The skill no longer uses a
> Bluetooth scanning tool.

## First run

Flash the chassis firmware, put the car on the floor with space around it (wheels off the ground for
the very first command), and read the name from the label or the Bluetooth list. The first action
asks for the last four characters; afterwards it does not ask again.

```text
@Honor RobotPhone Base 以 50 的速度前进 2 秒
@Honor RobotPhone Base 以 20 的速度前进 2 秒
@Honor RobotPhone Base 以 80 的速度前进 2 秒
@Honor RobotPhone Base 左轮 20、右轮 80，行驶 2 秒
@Honor RobotPhone Base 以 50 的速度前进 3 秒，然后以 50 的速度退后 3 秒
@Honor RobotPhone Base 打个招呼
@Honor RobotPhone Base 跳舞
@Honor RobotPhone Base 停止
```

The skill file is written against Chinese phrasing. The wording still does not have to match
exactly, but the matching happens in `description` and the intent table — so an English sentence
only works once it is in there. The wiki's Example 3 prompt asks an agent to add the English
equivalent beside every Chinese line; do the same by hand if you are not using an agent.

The firmware has no global auto-stop. A timed `speed` action ends with an explicit `stop`; an
untimed one keeps running until you say stop or the BLE link drops. Distance, angle and long
continuous motion are not encoder-calibrated — do not promise millimetres to anyone. Greeting, the
emotions and the dance open the gimbal first and then run the phone trajectory and the BLE command
in parallel; reverse park and dance also start the local music. Every combined action stops the car
by itself when it ends, and "stop" interrupts at any point.