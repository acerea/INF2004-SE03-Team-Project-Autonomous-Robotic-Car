# Subsystem 1 — test log

**Subsystem:** WiFi, MQTT and telemetry (comms)  ·  **Student ID:** 2502049  ·  **Code:** `app_program/subsystem1/`, `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | kernel objects exist (3 message buffers, 2 event flags) | IF-06, IF-07, IF-08 | auto | `subsystem1_test.c` | `[TEST] S1-U01 PASS` |
| S1-U02 | frame pack/unpack round trip at min and max values | wire format | auto | `subsystem1_test.c` | `[TEST] S1-U02 PASS` |
| S1-U03 | out-of-range value saturates and sets the error flag | wire format | auto | `subsystem1_test.c` | `[TEST] S1-U03 PASS` |
| S1-U04 | valid command accepted, duplicate rejected, STOP always accepted | IF-07 | auto | `subsystem1_test.c` | `[TEST] S1-U04 PASS` |
| S1-U05 | sequence window edges | IF-07 | auto | `subsystem1_test.c` (uses S1-U04 state, run in order) | `[TEST] S1-U05 PASS` |
| S1-U06 | bad version, bad command, bad length rejected | IF-07 | auto | `subsystem1_test.c` | `[TEST] S1-U06 PASS` |
| S1-U07 | full telemetry queue drops frames and never blocks (within 20 ms) | IF-06 | auto | `subsystem1_test.c` | `[TEST] S1-U07 PASS` |
| S1-U08 | frame type bits match the MQTT library | IF-06 | auto | `subsystem1_test.c` | `[TEST] S1-U08 PASS` |
| S1-M01 | Wi-Fi join and MQTT connect | comms | manual | 1. Hotspot on. 2. Flash normal image. 3. Read serial on COM5. | `wifi: link up`, `mqtt: connected`, `mqtt: subscribed to picocar-01/cmd` |
| S1-M02 | heartbeat published about every 2 s | comms | manual | 1. Run `mosquitto_sub -t "picocar-01/#" -v`. | `picocar-01/hb` line about every 2 s |
| S1-M03 | Last-Will published after link loss | link-loss | manual | 1. Unplug the Pico. 2. Watch `mosquitto_sub`. | one frame with `seq=255` on `picocar-01/hb` after about 15 s |
| S1-M04 | automatic reconnect | link-loss | manual | 1. Replug the Pico. | `mqtt: connected` and heartbeats resume |
| S1-M05 | command from laptop accepted by the car | IF-07 | manual | 1. Run `laptop_station.py`. 2. Type `stop`, then `start`. 3. Read serial on COM5. | `s1: cmd accepted cmd=1` then `cmd=0` appear |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-25 | S1-U01 | `c3b0954` + local changes | Kathryn | PASS | `logs/2026-10-25_unit.txt` | |
| 2026-10-25 | S1-U02 | `c3b0954` + local changes | Kathryn | PASS | `logs/2026-10-25_unit.txt` | |
| 2026-10-25 | S1-U03 | `c3b0954` + local changes | Kathryn | PASS | `logs/2026-10-25_unit.txt` | |
| 2026-10-25 | S1-U04 | `c3b0954` + local changes | Kathryn | PASS | `logs/2026-10-25_unit.txt` | |
| 2026-10-25 | S1-U05 | `c3b0954` + local changes | Kathryn | PASS | `logs/2026-10-25_unit.txt` | Depends on S1-U04 state; runs in order |
| 2026-10-25 | S1-U06 | `c3b0954` + local changes | Kathryn | PASS | `logs/2026-10-25_unit.txt` | |
| 2026-10-25 | S1-U07 | `c3b0954` + local changes | Kathryn | PASS | `logs/2026-10-25_unit.txt` | |
| 2026-10-25 | S1-U08 | `c3b0954` + local changes | Kathryn | PASS | `logs/2026-10-25_unit.txt` | Ran before MQTT connected; test checks constants only |
| 2026-10-25 | S1-M01 | `c3b0954` + local changes | Kathryn | PASS | serial log (wifi: link up, mqtt: connected, subscribed) | ip 192.168.137.33 |
| 2026-10-25 | S1-M02 | `c3b0954` + local changes | Kathryn | PASS | `logs/2026-10-25_heartbeat.png` | about 2 s interval |
| 2026-10-25 | S1-M03 | `c3b0954` + local changes | Kathryn | PASS | `logs/2026-10-25_lastwill_decoded.jpg`, `logs/2026-10-25_lastwill.jpg` | Last-Will frame (seq=255) published by broker about 15 s after Pico unplugged; blank before the ┴ is the 0xFF sequence byte |
| 2026-10-25 | S1-M04 | `c3b0954` + local changes | Kathryn | PASS | `logs/2026-10-25_heartbeat.png` | heartbeats resumed after replug |
| 2026-10-25 | S1-M05 | `c3b0954` + local changes | Kathryn | PASS | `logs/2026-10-25_command_laptop.png`, `logs/2026-10-25_command_serial.png` | STOP (cmd=1) and START (cmd=0) accepted; sent as `000500` and `010100` |