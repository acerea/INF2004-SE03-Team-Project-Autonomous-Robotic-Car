# System — test log

Tests of the complete product against its requirements, run on the final hardware set-up. Run the full set before every demo and before hand-in.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| SYS-01 | Line following over the whole course | FR7, NFR10 | manual | 1. Calibrate on the line. 2. START. 3. Watch the car drive the course, including bends. | The car follows the line on straights and bends without manual help. |
| SYS-02 | Barcode detection and decoding | FR9, NFR12 | manual | 1. Barcode on the course. 2. START and let the car drive over it. 3. Watch BARCODE SCANS and LAST BARCODE on the dashboard. | A scan appears and the decoded letter matches the printed barcode. |
| SYS-03 | Barcode instruction carried out at the junction | FR8, FR10 | manual | 1. Set the letter's action on the dashboard (BARCODE ACTIONS). 2. START before a barcode followed by a T-junction. | `junction: doing <action>`; the car turns left/right, goes straight or U-turns as set, then carries on following the line. |
| SYS-04 | Hump height measurement | FR11, NFR9, NFR16 | manual | 1. 10 mm hump under the right wheel. 2. START and drive over it. 3. Read the IMU tile. | Hump measured within 1 cm of 10 mm; no hump reported on flat ground. |
| SYS-05 | Obstacle detection | FR13, NFR7 | manual | 1. Obstacle on the line. 2. START. | The car stops in front of the obstacle without touching it (OBSTACLE tile EMERGENCY, state BLOCKED). |
| SYS-06 | Obstacle width measurement | FR14, NFR15 | manual | 1. Measure the obstacle's width with a ruler. 2. Run SYS-05. 3. Read OBSTACLE WIDTH on the dashboard. | Width within 20 % of the ruler, or shown as a minimum ("≥ N cm") when the obstacle is wider than the sweep can see. |
| SYS-07 | Obstacle bypass and line rejoin | FR19, NFR8 | manual | Continue from SYS-05. | The car goes round the obstacle without touching it, `bypass done: middle sensor on the line`, and carries on along the line. |
| SYS-08 | Telemetry to the dashboard over MQTT | FR1, FR21, NFR4, NFR18 | manual | 1. Run `station.py` and the dashboard. 2. START and drive for 1 minute. | Tiles update during the run; STATE at least every 1 s and heartbeat at least every 2 s in the station log. |
| SYS-09 | Remote START and STOP | FR2, FR16, NFR5 | manual | 1. START from the dashboard. 2. Press STOP while driving. | The car starts on START and stops at once on STOP. |
| SYS-10 | Stop when picked up (end of run) | FR12, FR16 | manual | While the car drives, pick it up firmly. | Wheels stop at once; `robot lifted, motors stopped`; state STOPPED. |
| SYS-11 | Stop when the laptop link is lost | FR16, NFR14 | manual | 1. START. 2. Close `station.py` while the car drives. | The car stops within about 2 s. |
| SYS-12 | Stop when overturned | FR16, FR18 | manual | While driving, tip the car past 60 degrees (or upside down). | Motors stop, state FAULT until RESET. |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | SYS-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-11-26 | SYS-01 | `a58b39b` + bypass pivot fix in `vehicle.c` (uncommitted) | Kathryn | PASS | `logs/2026-11-26_R5_station.log`, `logs/2026-11-26_R5_dashboard.log`, `logs/2026-11-26_R5_dashboard.png`, R5 <https://youtu.be/o11OrEEbCXo> | Drove the course including bends without manual help; brief line losses recovered by itself. |
| 2026-11-26 | SYS-02 | `a58b39b` + bypass pivot fix in `vehicle.c` (uncommitted) | Kathryn | PASS | `logs/2026-11-26_R5_station.log`, `logs/2026-11-26_R5_dashboard.log`, `logs/2026-11-26_R5_dashboard.png`, R5 <https://youtu.be/o11OrEEbCXo> | Barcode scanned at 13:39:42 (29 elements, OK) and decoded as W. |
| 2026-11-26 | SYS-04 | `a58b39b` + bypass pivot fix in `vehicle.c` (uncommitted) | Kathryn | PASS | `logs/2026-11-26_R5_station.log`, `logs/2026-11-26_R5_dashboard.log`, `logs/2026-11-26_R5_dashboard.png`, R5 <https://youtu.be/o11OrEEbCXo> | 10 mm hump measured 12 mm and 11 mm (within 1 cm). |
| 2026-11-26 | SYS-05 | `a58b39b` + bypass pivot fix in `vehicle.c` (uncommitted) | Kathryn | PASS | `logs/2026-11-26_R5_station.log`, `logs/2026-11-26_R5_dashboard.log`, `logs/2026-11-26_R5_dashboard.png`, R5 <https://youtu.be/o11OrEEbCXo> | Stopped in front of the obstacle (EMERGENCY, BLOCKED) without touching it. |
| 2026-11-26 | SYS-06 | `a58b39b` + bypass pivot fix in `vehicle.c` (uncommitted) | Kathryn | PASS | `logs/2026-11-26_R5_station.log`, `logs/2026-11-26_R5_dashboard.log`, `logs/2026-11-26_R5_dashboard.png`, R5 <https://youtu.be/o11OrEEbCXo> | Width shown as a minimum (>= 18 cm): the obstacle was wider than the sweep can see on both sides. |
| 2026-11-26 | SYS-07 | `a58b39b` + bypass pivot fix in `vehicle.c` (uncommitted) | Kathryn | PASS | `logs/2026-11-26_R5_station.log`, `logs/2026-11-26_R5_dashboard.log`, `logs/2026-11-26_R5_dashboard.png`, R5 <https://youtu.be/o11OrEEbCXo> | Two bypasses, both `bypass done: middle sensor on the line`, then carried on along the line. |
| 2026-11-26 | SYS-08 | `a58b39b` + bypass pivot fix in `vehicle.c` (uncommitted) | Kathryn | PASS | `logs/2026-11-26_R5_station.log`, `logs/2026-11-26_R5_dashboard.log`, `logs/2026-11-26_R5_dashboard.png`, R5 <https://youtu.be/o11OrEEbCXo> | Dashboard updated throughout; STATE and heartbeat frames received regularly in the station log. |
| 2026-11-26 | SYS-10 | `a58b39b` + bypass pivot fix in `vehicle.c` (uncommitted) | Kathryn | PASS | `logs/2026-11-26_R5_station.log`, `logs/2026-11-26_R5_dashboard.log`, `logs/2026-11-26_R5_dashboard.png`, R5 <https://youtu.be/o11OrEEbCXo> | Picked up at 13:41:19: `robot lifted, motors stopped`, state STOPPED. |
| 2026-11-26 | SYS-03 | `a58b39b` + junction fixes and U-turn 200 deg in `vehicle.c` (uncommitted) | Kathryn | FAIL | `../integration/logs/2026-11-26_R9_station.log`, R9 <https://youtube.com/shorts/5Fv-NCRoDgM> | U-turn at the junction (200 deg) did not end straight on the line. |
| 2026-11-26 | SYS-03 | `a58b39b` + junction fixes in `vehicle.c` (uncommitted) | Kathryn | PASS | `../integration/logs/2026-11-26_R6_station.log`, R6 <https://youtube.com/shorts/doQBv0pJ9EA>, R7 <https://youtube.com/shorts/iOsTTYroVC4>, R8 <https://youtube.com/shorts/mdN2Bu_AWHk> | Barcode W carried out at the T-junction as set: LEFT (R6), RIGHT (R7), STRAIGHT (R8); then carried on following the line. |
| 2026-11-26 | SYS-02 | `a58b39b` + junction fixes in `vehicle.c` (uncommitted) | Kathryn | PASS | `../integration/logs/2026-11-26_R6_station.log`, R6 <https://youtube.com/shorts/doQBv0pJ9EA>, R7 <https://youtube.com/shorts/iOsTTYroVC4>, R8 <https://youtube.com/shorts/mdN2Bu_AWHk> | Barcode W decoded correctly in R6, R7 and R8. |
