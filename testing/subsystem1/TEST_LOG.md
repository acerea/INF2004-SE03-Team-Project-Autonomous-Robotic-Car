# Subsystem 1 — test log

**Subsystem:** WiFi, MQTT and telemetry (comms)  ·  **Student ID:** 2502049  ·  **Code:** `app_program/subsystem1/`, `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | kernel objects exist (3 message buffers, 2 event flags) | NFR11, IF-06, IF-07, IF-08 | auto | `subsystem1_test.c` | `[TEST] S1-U01 PASS` |
| S1-U02 | frame pack/unpack round trip at min and max values | CR, wire format | auto | `subsystem1_test.c` | `[TEST] S1-U02 PASS` |
| S1-U03 | out-of-range value saturates and sets the error flag | CR, wire format | auto | `subsystem1_test.c` | `[TEST] S1-U03 PASS` |
| S1-U04 | valid command accepted, duplicate rejected, STOP always accepted | FR2, IF-07 | auto | `subsystem1_test.c` | `[TEST] S1-U04 PASS` |
| S1-U05 | sequence window edges | FR2, IF-07 | auto | `subsystem1_test.c` (uses S1-U04 state, run in order) | `[TEST] S1-U05 PASS` |
| S1-U06 | bad version, bad command, bad length rejected | FR2, IF-07 | auto | `subsystem1_test.c` | `[TEST] S1-U06 PASS` |
| S1-U07 | full telemetry queue drops frames and never blocks (within 20 ms) | NFR4, NFR11, IF-06 | auto | `subsystem1_test.c` | `[TEST] S1-U07 PASS` |
| S1-U08 | frame type bits match the MQTT library | FR1, IF-06 | auto | `subsystem1_test.c` | `[TEST] S1-U08 PASS` |
| S1-U09 | 1-byte frame (oversize marker) rejected by the length check | FR2, IF-07 | auto | `subsystem1_test.c` | `[TEST] S1-U09 PASS` |
| S1-U10 | command counters wrap 255 to 0 and fit their 8-bit heartbeat fields | FR2, wire format | auto | `subsystem1_test.c` | `[TEST] S1-U10 PASS` |
| S1-U11 | uptime field counts whole seconds, wraps at 65536 s and fits 16 bits | FR1, wire format | auto | `subsystem1_test.c` | `[TEST] S1-U11 PASS` |
| S1-M01 | Wi-Fi join and MQTT connect | FR1, comms | manual | 1. Hotspot on. 2. Flash normal image. 3. Read serial on COM5. | `wifi: link up`, `mqtt: connected`, `mqtt: subscribed to picocar-01/cmd` |
| S1-M02 | heartbeat published about every 2 s | FR1, NFR4, comms | manual | 1. Run `mosquitto_sub -t "picocar-01/#" -v`. | `picocar-01/hb` line about every 2 s |
| S1-M03 | Last-Will published after link loss | FR21, link-loss | manual | 1. Unplug the Pico. 2. Watch `mosquitto_sub`. | one frame with `seq=255` on `picocar-01/hb` after about 15 s |
| S1-M04 | automatic reconnect | FR3, link-loss | manual | 1. Replug the Pico. | `mqtt: connected` and heartbeats resume |
| S1-M05 | command from laptop accepted by the car | FR2, IF-07 | manual | 1. Run `laptop_station.py`. 2. Type `stop`, then `start`. 3. Read serial on COM5. | `s1: cmd accepted cmd=1` then `cmd=0` appear |
| S1-M06 | command counters in the heartbeat | FR2, IF-07, wire format | manual | 1. Run `mosquitto_sub -t "picocar-01/#" -v` and note cmd_ok and cmd_bad in a heartbeat. 2. In `laptop_station.py` send STOP, START, one bad-version 3-byte frame and one 5-byte frame. 3. Read the next heartbeat. | cmd_ok up by 2, cmd_bad up by 2 |
| S1-M07 | heartbeat interval at most 2 s | NFR4 | manual | 1. Flash normal image and wait for heartbeats. 2. In PowerShell run `mosquitto_sub -t "picocar-01/hb" -C 100 -F "%x"`, add a laptop timestamp to each line with `ForEach-Object` and `[DateTimeOffset]::Now.ToUnixTimeMilliseconds()`, and save it with `Tee-Object`. 3. Compute the gaps between heartbeats, ignoring Last-Will frames (seq 255). | every gap at most 2000 ms; mean period from the uptime field at most 2.0 s |
| S1-M08 | broker loss: LOST, FAULT after 10 s, automatic recovery | FR3, NFR6 | manual | 1. Pico connected; serial monitor open. 2. In an Administrator PowerShell run `net stop mosquitto`. 3. Short run: `net start mosquitto` before FAULT. Long run: wait for `LOST -> FAULT`, then `net start mosquitto`. 4. Read the `s1:` and `mqtt:` lines. | `CONNECTED -> LOST` within 5 s of the last broker reply; `LOST -> FAULT` 10 s after LOST; `-> CONNECTED` after the broker restarts, with no reset |
| S1-M09 | Wi-Fi drop: LOST, FAULT after 10 s, reconnect within 10 s of the network returning | FR3, NFR6 | manual | 1. Pico connected; PowerShell serial logger running (VS Code serial monitor closed). 2. Switch the laptop hotspot off. 3. After `-> FAULT`, switch it on. 4. Read the `s1:`, `wifi:` and `mqtt:` lines. | `CONNECTED -> LOST` within 5 s; FAULT 10 s after LOST; `-> CONNECTED` within 10 s of the hotspot being on, with no reset |
| S1-M10 | message size: binary frames vs an equivalent text encoding of the same fields | CR, FR1, FR2 | manual | 1. Use a heartbeat capture (as in S1-M07). 2. Run `python3 logs/msg_size.py <capture>`. 3. Compare payload and MQTT packet sizes. | binary payload and packet smaller than key=value and JSON text for heartbeat and command |
| S1-M11 | RAM and size of subsystem 1 data vs the natural representation | CR | manual | 1. Build the test image. 2. Run `arm-none-eabi-size` on the subsystem 1 and `mqtt_utk` object files. 3. Compare each field with its natural C size. | every field uses fewer or equal bytes than its natural representation |
| S1-M12 | runs with no USB computer attached: Wi-Fi, MQTT, heartbeat and commands | FR1, FR2 | manual | 1. Flash normal image. 2. Power the Pico from a phone charger (no computer). 3. Run `laptop_station.py` and send `stop`. | heartbeats with `link=CONNECTED` after power-on; `cmd_ok` up by 1 after `stop` |


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
| 2026-10-27 | S1-U01 | `227e948` + local changes | Kathryn | PASS | `logs/2026-10-27_unit.jpg` | Start of log not captured (monitor opened after boot) |
| 2026-10-27 | S1-U02 | `227e948` + local changes | Kathryn | PASS | `logs/2026-10-27_unit.jpg` | |
| 2026-10-27 | S1-U03 | `227e948` + local changes | Kathryn | PASS | `logs/2026-10-27_unit.jpg` | |
| 2026-10-27 | S1-U04 | `227e948` + local changes | Kathryn | PASS | `logs/2026-10-27_unit.jpg` | |
| 2026-10-27 | S1-U05 | `227e948` + local changes | Kathryn | PASS | `logs/2026-10-27_unit.jpg` | Depends on S1-U04 state; runs in order |
| 2026-10-27 | S1-U06 | `227e948` + local changes | Kathryn | PASS | `logs/2026-10-27_unit.jpg` | |
| 2026-10-27 | S1-U07 | `227e948` + local changes | Kathryn | PASS | `logs/2026-10-27_unit.jpg` | |
| 2026-10-27 | S1-U08 | `227e948` + local changes | Kathryn | PASS | `logs/2026-10-27_unit.jpg` | Ran before MQTT connected; test checks constants only |
| 2026-10-27 | S1-U09 | `227e948` + local changes | Kathryn | PASS | `logs/2026-10-27_unit.jpg` | New test; 1-byte oversize marker rejected, sequence unchanged. Code includes oversize-marker change in mqtt_utk.c |
| 2026-10-27 | S1-M01 | `227e948` + local changes | Kathryn | PASS | `logs/2026-10-27_command_serial.png` | ip 192.168.137.47; built with S1_DEBUG_CMD=1 |
| 2026-10-27 | S1-M05 | `227e948` + local changes | Kathryn | PASS | `logs/2026-10-27_command_laptop.png`, `logs/2026-10-27_command_serial.png` | STOP (cmd=1) and START (cmd=0) accepted; sent as `000500` and `010100`; built with S1_DEBUG_CMD=1 |
| 2026-10-27 | S1-M02 | `227e948` + local changes | Kathryn | PASS | `logs/2026-10-27_heartbeat_lastwill.png` | about 2 s interval; 8-byte frames (16 hex digits), seq counts up; built with S1_DEBUG_CMD=1 |
| 2026-10-27 | S1-M03 | `227e948` + local changes | Kathryn | PASS | `logs/2026-10-27_heartbeat_lastwill.png` | Last-Will ffc1ffffffffff01 (seq=255) published about 16 s after the last heartbeat (15:20:46 to 15:21:02) |
| 2026-10-27 | S1-M04 | `227e948` + local changes | Kathryn | PASS | `logs/2026-10-27_reconnect.png` | Pico replugged; heartbeats resumed at 15:23:08 with seq restarting at 00|
| 2026-10-27 | S1-M06 | `227e948` + local changes | Kathryn | PASS | `logs/2026-10-27_cmd_counters.jpg` | After reboot, heartbeat bytes 5 and 6 start at 00 00. After STOP and START byte 5 goes 00 to 04 and byte 6 stays 00. After 3-byte `abc` and 5-byte `abcde` byte 6 goes 00 to 04 and byte 5 stays 04. So cmd_ok +2 and cmd_bad +2; built with S1_DEBUG_CMD=1 |
| 2026-10-29 | S1-U01 | `007ae27` | Kathryn | PASS | `logs/2026-10-29_unit.jpg` | Re-run after merge of origin/main (`9a94a38`) |
| 2026-10-29 | S1-U02 | `007ae27` | Kathryn | PASS | `logs/2026-10-29_unit.jpg` | |
| 2026-10-29 | S1-U03 | `007ae27` | Kathryn | PASS | `logs/2026-10-29_unit.jpg` | |
| 2026-10-29 | S1-U04 | `007ae27` | Kathryn | PASS | `logs/2026-10-29_unit.jpg` | |
| 2026-10-29 | S1-U05 | `007ae27` | Kathryn | PASS | `logs/2026-10-29_unit.jpg` | Depends on S1-U04 state; runs in order |
| 2026-10-29 | S1-U06 | `007ae27` | Kathryn | PASS | `logs/2026-10-29_unit.jpg` | |
| 2026-10-29 | S1-U07 | `007ae27` | Kathryn | PASS | `logs/2026-10-29_unit.jpg` | |
| 2026-10-29 | S1-U08 | `007ae27` | Kathryn | PASS | `logs/2026-10-29_unit.jpg` | Test checks constants only |
| 2026-10-29 | S1-U09 | `007ae27` | Kathryn | PASS | `logs/2026-10-29_unit.jpg` | 1-byte oversize marker rejected, sequence unchanged |
| 2026-11-03 | S1-M07 | `d19e050` + local change | Kathryn | FAIL | `logs/2026-11-03_hb_interval_before.txt` | Heartbeat set to 1900 ms but the 100 ms tick drifted to about 104 ms. 99 gaps: min 464, max 3477, mean 1983.3 ms, 28 over 2000 ms; uptime mean 1.98 s. Fixed in `bb258ed` |
| 2026-11-03 | S1-M07 | `bb258ed` | Kathryn | PASS | `logs/2026-11-03_hb_interval_after.txt` | 98 gaps, Last-Will frame excluded: min 1797, max 1995, mean 1899.7 ms, 0 over 2000 ms; uptime 186 s over 98 steps = 1.898 s. Gaps measured at the laptop, so they include Wi-Fi delay |
| 2026-11-03 | S1-M08 | `0850bc5` | Kathryn | PASS | `logs/2026-11-03_broker_stop_short.txt` | Short outage: LOST at 25000 ms, broker restarted before FAULT, CONNECTED at 33400 ms (8.4 s in LOST, no FAULT) |
| 2026-11-03 | S1-M08 | `0850bc5` | Kathryn | PASS | `logs/2026-11-03_broker_stop_long.txt` | Long outage: LOST at 152200 ms, FAULT at 162200 ms (10.0 s), CONNECTED at 169300 ms after broker restart, no reset |
| 2026-11-03 | S1-M09 | `0850bc5` | Kathryn | FAIL | `logs/2026-11-03_wifi_drop_runs1-3.txt` (runs 1-3) | Comms side passed every run: LOST within 5 s, FAULT 10.0 s after LOST. Recovery: 11.77 s (run 1), none within 60 s, reset needed (run 2), about 17 s (run 3). The template's Wi-Fi rejoin retry never appeared |
| 2026-11-03 | S1-M09 | `0850bc5` | Kathryn | FAIL | `logs/2026-11-03_wifi_drop_timer.txt` (timer check, runs 4-5) | Cause in template: `time_us_64()` advances 23672 us per 1 s (about 42x slow; WATCHDOG_TICK never set), so the 5 s rejoin delay is 214 s (measured 4 times). Rejoin also failed authentication when the laptop hotspot had no internet; succeeded when it shared phone internet |
| 2026-11-03 | S1-M09 | `0850bc5` + uncommitted WATCHDOG_TICK fix | Kathryn | PASS | `logs/2026-11-03_wifi_drop_tickfix.txt` (run 6) | With `WATCHDOG_TICK = 0x200 \| 12` at boot, the rejoin retried 5 s after the failure, then every 6 s; reconnected 3 s after the retry. Template change pending instructor approval |
| 2026-11-03 | S1-U01 | `3338bca` | Kathryn | PASS | `logs/2026-11-03_unit.jpg` | Test build. Re-run of all unit tests after the uptime and counter helpers were added; heartbeat uptime and cmd_ok checked live with laptop_station.py |
| 2026-11-03 | S1-U02 | `3338bca` | Kathryn | PASS | `logs/2026-11-03_unit.jpg` | |
| 2026-11-03 | S1-U03 | `3338bca` | Kathryn | PASS | `logs/2026-11-03_unit.jpg` | |
| 2026-11-03 | S1-U04 | `3338bca` | Kathryn | PASS | `logs/2026-11-03_unit.jpg` | |
| 2026-11-03 | S1-U05 | `3338bca` | Kathryn | PASS | `logs/2026-11-03_unit.jpg` | Depends on S1-U04 state; runs in order |
| 2026-11-03 | S1-U06 | `3338bca` | Kathryn | PASS | `logs/2026-11-03_unit.jpg` | |
| 2026-11-03 | S1-U07 | `3338bca` | Kathryn | PASS | `logs/2026-11-03_unit.jpg` | |
| 2026-11-03 | S1-U08 | `3338bca` | Kathryn | PASS | `logs/2026-11-03_unit.jpg` | Test checks constants only |
| 2026-11-03 | S1-U09 | `3338bca` | Kathryn | PASS | `logs/2026-11-03_unit.jpg` | |
| 2026-11-03 | S1-U10 | `3338bca` | Kathryn | PASS | `logs/2026-11-03_unit.jpg` | New. Boundaries: 0 to 1, 254 to 255, 255 to 0; results fit the 8-bit fields with no error flag |
| 2026-11-03 | S1-U11 | `3338bca` | Kathryn | PASS | `logs/2026-11-03_unit.jpg` | New. Boundaries: 0.999 s to 0, 1.000 s to 1, 65535.999 s to 65535, 65536 s to 0, UINT32_MAX ms fits 16 bits |
| 2026-11-03 | S1-U01 | `09f2196` | Kathryn | PASS | `logs/2026-11-03_unit_after_merge.jpg` | Test build. Re-run after merge of origin/main (`1f440e9`) |
| 2026-11-03 | S1-U02 | `09f2196` | Kathryn | PASS | `logs/2026-11-03_unit_after_merge.jpg` |  |
| 2026-11-03 | S1-U03 | `09f2196` | Kathryn | PASS | `logs/2026-11-03_unit_after_merge.jpg` |  |
| 2026-11-03 | S1-U04 | `09f2196` | Kathryn | PASS | `logs/2026-11-03_unit_after_merge.jpg` |  |
| 2026-11-03 | S1-U05 | `09f2196` | Kathryn | PASS | `logs/2026-11-03_unit_after_merge.jpg` |  |
| 2026-11-03 | S1-U06 | `09f2196` | Kathryn | PASS | `logs/2026-11-03_unit_after_merge.jpg` |  |
| 2026-11-03 | S1-U07 | `09f2196` | Kathryn | PASS | `logs/2026-11-03_unit_after_merge.jpg` |  |
| 2026-11-03 | S1-U08 | `09f2196` | Kathryn | PASS | `logs/2026-11-03_unit_after_merge.jpg` |  |
| 2026-11-03 | S1-U09 | `09f2196` | Kathryn | PASS | `logs/2026-11-03_unit_after_merge.jpg` |  |
| 2026-11-03 | S1-U10 | `09f2196` | Kathryn | PASS | `logs/2026-11-03_unit_after_merge.jpg` |  |
| 2026-11-03 | S1-U11 | `09f2196` | Kathryn | PASS | `logs/2026-11-03_unit_after_merge.jpg` |  |
| 2026-11-03 | S1-M10 | `bb258ed` (capture) | Kathryn | PASS | `logs/2026-11-03_msg_size.txt` | Heartbeat (99 real frames): 8 B vs 51.4 B key=value / 69.4 B JSON; MQTT packet 27 vs 70.4 / 88.4 B. Command (all 20480 values): 3 B vs 24.9 / 34.9 B; packet 23 vs 44.9 / 54.9 B. Tool: `logs/msg_size.py` |
| 2026-11-03 | S1-M11 | `6a3ecdf` | Kathryn | PASS | `logs/2026-11-03_ram_usage.md` | Comms 2356 B code, 38 B static RAM. Context struct 32 B vs 52 B; queued command 1 B vs 8 B; buffers hold 2x to 4x more messages. No "before" build exists (bit-packed from the first commit) |
| 2026-11-03 | S1-M12 | `b85f027` | Kathryn | PASS | `logs/2026-11-03_no_usb_host.jpg` | Phone charger, no computer. Last-Will on unplug. After replug the first join failed and the first heartbeat came at uptime 228 s (slow template timer, see S1-M09); the previous boot connected at 14 s. `stop` gave cmd_ok 0 to 1. A USB-C laptop charger with a micro-USB adapter did not power the Pico |
