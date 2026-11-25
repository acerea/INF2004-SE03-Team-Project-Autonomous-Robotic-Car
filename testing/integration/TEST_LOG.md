# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-VC1 | Vehicle Controller stage 1: remote START / STOP / ESTOP / RESET change the mission state and motion request | FR2, FR15, FR16, IF-07 | manual | 1. Flash normal image. 2. Serial logger on COM5 (vc: lines). 3. In `laptop_station.py` type start, stop, start, estop, start, reset. | IDLE -> FOLLOWING (speed request) -> STOPPED (stop request) -> FOLLOWING -> FAULT; START ignored in FAULT; RESET -> IDLE |
| INT-VC2 | Vehicle state telemetry on the MQTT dashboard | FR1, FR21 | manual | 1. Run `laptop_station.py` and MQTT Explorer (localhost:1883). 2. Type start, then stop. 3. Open `picocar-01/view/state` and its History. | STATE about every 1 s; state changes appear at once; History shows IDLE, FOLLOWING, STOPPED |
| INT-VC4 | VC motion requests reach subsystem 2 | VC -> S2 (`mbf_motion_req`) | manual | 1. Flash normal image. 2. Serial logger on COM5. 3. In `laptop_station.py` type start, stop, start, estop, start, reset. | `vc: motion cmd=1 arg=20` on start, `cmd=0` on stop/estop; no `busy, retrying`; on the car: wheels spin on start, stop on stop/estop |
| INT-VC5 | Obstacle stops the car and the sweep picks the bypass side | IF-06 (S5 -> VC) | manual | Run R1: box on the straight, sitting mostly on one side of the line. Watch the dashboard events and serial `[OBS]` lines. | Car stops ~15 cm before the box (BLOCKED); `sweep: width ... -> go RIGHT` for a box on the left (LEFT for a box on the right); `bypass right/left` event with the same side. |
| INT-VC6 | Bypass legs are driven by S2 and each finish starts the next | IF-11 (VC -> S2), IF-12 (S2 -> VC) | manual | Run R1, same bypass. | Car turns out, passes, turns in and finds the line; events `bypass: line seen N cm into the way back` and `bypass done: middle sensor on the line`; state BYPASSING -> FOLLOWING. |
| INT-VC7 | Line telemetry steers the car, including curves | IF-14 (S3 -> VC) | manual | Run R1: full course with straight, left and right bends. | Car follows the line on all bends; LINE tile on the dashboard shows the sensors and position changing; no `line not found` hold. |
| INT-VC8 | Barcode read is executed at the next T-junction | IF-15 (S3 -> VC) | manual | Before R1, set the barcode letter's action on the dashboard (e.g. A -> LEFT). Barcode before a T-junction. | `barcode A: LEFT at the next junction`, then `junction: doing LEFT`; car turns that way at the junction; BARCODE SCANS shows the read. |
| INT-VC9 | Hump height measured and shown on the dashboard | IF-10 (S4 -> S1) | manual | Run R1 over the 10 mm hump (right wheel). | `hump started` event, then the IMU tile shows the hump at 10 mm +/- 2 mm. |
| INT-VC10 | Picking the car up stops the motors (end of demo) | IF-13 (S4 -> VC) | manual | End of R1: firmly lift the car while it drives. | Wheels stop at once; event `robot lifted, motors stopped`. |
| INT-VC11 | Telemetry from every subsystem reaches the dashboard over MQTT | IF-09, IF-10 (all -> S1 -> MQTT) | manual | During R1, check the station log. | STATE, LINE, OBSTACLE, OBST_PROFILE, TERRAIN and EVENT frames all present in `logs/station_*.log`; tiles update. |
| INT-VC12 | Laptop link lost stops the car | IF-08 (S1 -> VC) | manual | Run R2: start the car, then close `station.py` (Ctrl-C) while it drives. Restart the station afterwards. | Car stops within about 2 s; after restarting the station, event `laptop link lost (... ms without a command), stopping` appears in the log. |
| INT-VC13 | Dashboard nudge moves the car forward and back | IF-11 (VC -> S2), IF-12 (S2 -> VC) | manual | Run R3: car IDLE; NUDGE forward 10 cm, then back 10 cm. Measure with a ruler. | Moves 10 cm +/- 1 cm each way at cruise speed; `nudge forward/back 10 cm` events; refused while on a run. |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | INT-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-11-11 | INT-VC1 | `cb98e5e` + local S3 workaround | Kathryn | PASS | `logs/2026-11-11_vc_stage1.txt` | All transitions as expected; motion requests printed (stub) until subsystem 2 provides its request buffer |
| 2026-11-12 | INT-VC1 | `b2d2262` + local S3 workaround | Kathryn | PASS | `logs/2026-11-12_vc_switch.txt` | Re-run after refactor to switch(state) event dispatch: same transitions; START ignored in FAULT |
| 2026-11-12 | INT-VC4 | `111c194` + local S3 workaround | Kathryn | PASS (bench) | `logs/2026-11-12_vc_motion_req.txt` | Bench Pico without motors: every request accepted by S2 at once. Wheel response on the car still to test |
| 2026-11-11 | INT-VC2 | `9710c67` + local S3 workaround | Kathryn | PASS | `../subsystem1/logs/2026-11-11_mqtt_explorer.png` | STATE about 1 s apart after the clock-based fix; IDLE -> FOLLOWING -> STOPPED visible in History |
| 2026-11-25 | INT-VC5 | `f2b1748` | Kathryn | PASS | `logs/2026-11-25_R1_station.log`, `logs/2026-11-25_R2_station.log`, R1 <https://youtu.be/KNKkJJi9sS0>, R2 <https://youtu.be/FVoq1ijGTDE> | Stopped at 13 cm (BLOCKED); sweep reported width >= 16-17 cm, wider than the sweep on both sides, so the default side RIGHT was chosen; bypass went right as reported. |
| 2026-11-25 | INT-VC6 | `f2b1748` | Kathryn | PASS | `logs/2026-11-25_R1_station.log`, `logs/2026-11-25_R2_station.log`, `logs/2026-11-25_R3_station.log`, R1 <https://youtu.be/KNKkJJi9sS0>, R2 <https://youtu.be/FVoq1ijGTDE>, R3 <https://youtu.be/xRuwYrglJrI> | All three bypasses ended `bypass done: middle sensor on the line`, BYPASSING -> FOLLOWING (R1: line seen 8 cm into the way back). |
| 2026-11-25 | INT-VC7 | `f2b1748` | Kathryn | PASS | `logs/2026-11-25_R1_dashboard.log`, `logs/2026-11-25_R2_dashboard.log`, `logs/2026-11-25_R3_dashboard.log`, R1 <https://youtu.be/KNKkJJi9sS0>, R2 <https://youtu.be/FVoq1ijGTDE>, R3 <https://youtu.be/xRuwYrglJrI> | Followed the course including bends in all three runs; no `line not found` hold (R3: `line found again` once). |
| 2026-11-25 | INT-VC9 | `f2b1748` | Kathryn | PASS | `logs/2026-11-25_R1_dashboard.log`, `logs/2026-11-25_R2_dashboard.log`, `logs/2026-11-25_R3_dashboard.log`, R1 <https://youtu.be/KNKkJJi9sS0>, R2 <https://youtu.be/FVoq1ijGTDE>, R3 <https://youtu.be/xRuwYrglJrI> | Hump #1 measured 11 mm in each run (10 mm hump, within +/- 2 mm). |
| 2026-11-25 | INT-VC10 | `f2b1748` | Kathryn | PASS | `logs/2026-11-25_R3_dashboard.log`, R3 <https://youtu.be/xRuwYrglJrI> | Picked up while following at 20:54:36: LIFTED, `robot lifted, motors stopped`, FOLLOWING -> STOPPED. Needs a firm lift (known limitation, see B4-H05). |
| 2026-11-25 | INT-VC11 | `f2b1748` | Kathryn | PASS | `logs/2026-11-25_R1_station.log`, `logs/2026-11-25_R2_station.log`, `logs/2026-11-25_R3_station.log` | STATE, LINE, MOTION, OBSTACLE, TERRAIN, EVENT and heartbeat frames in every run; OBST_PROFILE in R1/R2 (R3's was lost in a Wi-Fi drop). |
