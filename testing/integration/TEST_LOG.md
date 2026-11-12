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
