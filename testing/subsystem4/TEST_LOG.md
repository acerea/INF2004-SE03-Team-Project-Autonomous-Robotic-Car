# Subsystem 4 — test log

**Subsystem:** Monitor (example)  ·  **Student ID:** <student ID>  2503813  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
<!-- | S4-U01 | message buffer max line = S4_SUMMARY_MAX | IF-03 | auto | `subsystem4_test.c` | `[TEST] S4-U01 PASS` |
| S4-U02 | LED pauses and resumes every 6 reports | IF-05 | manual | 1. Flash normal image. 2. Watch LED and console ~20 s. | console shows "LED task paused"/"resumed"; LED stops/starts | -->

| B4-U01 | Pack a stationary motion state into a compact event | Compact event representation | auto | subsystem4_test.c | Event contains the correct stationary state and no additional flags. |
| B4-U02 | Pack a moving motion state into a compact event | Compact event representation | auto | subsystem4_test.c | Event contains the correct moving state. |
| B4-U03 | Pack a collision flag into an event | Collision event representation | auto | subsystem4_test.c | Collision bit (bit 3) is correctly set and can be decoded. |
| B4-U04 | Pack an IMU fault flag into an event | IMU fault representation | auto | subsystem4_test.c | IMU fault bit (bit 4) is correctly set and can be decoded. |
| B4-U05 | Classify stationary motion | Motion classification | auto | subsystem4_test.c | Low motion activity is classified as STATIONARY. |
| B4-U06 | Classify moving motion | Motion classification | auto | subsystem4_test.c | Motion activity above the configured threshold is classified as MOVING. |
| B4-U07 | Classify positive yaw-rate turning | Motion classification | auto | subsystem4_test.c | Positive yaw-rate input above the turning threshold is classified as TURNING. |
| B4-U08 | Prevent tilt alone from being classified as climbing | Motion classification | auto | subsystem4_test.c | Tilt without sufficient motion evidence does not automatically produce CLIMBING. |
| B4-U09 | Prevent tilt alone from being classified as descending | Motion classification | auto | subsystem4_test.c | Tilt without sufficient motion evidence does not automatically produce DESCENDING. |
| B4-U10 | Detect impact and collision from acceleration change | Impact and collision detection | auto | subsystem4_test.c | Impact delta at or above the configured threshold produces an IMPACT state and sets the collision flag. |
| B4-U11 | Detect IMU fault condition | IMU fault handling | auto | subsystem4_test.c | IMU fault input produces FAULT state and sets the IMU fault flag. |
| B4-U12 | Test motion threshold boundary | Motion classification threshold | auto | subsystem4_test.c | Motion activity at the configured threshold is classified according to the threshold condition. |
| B4-U13 | Test motion activity below threshold | Motion classification threshold | auto | subsystem4_test.c | Motion activity below the configured threshold is not classified as MOVING. |
| B4-U14 | Classify negative yaw-rate turning | Motion classification | auto | subsystem4_test.c | Negative yaw-rate input above the turning threshold magnitude is classified as TURNING. |
| B4-U15 | Verify IMU fault classification priority | Motion classification and fault handling | auto | subsystem4_test.c | IMU fault takes priority over other motion conditions. |
| B4-U16 | Verify reserved bits remain unused when packing an event | Compact event representation | auto | subsystem4_test.c | Bits 5–7 remain zero after packing a turning event with collision and IMU fault flags. |
| B4-R01 | Verify 30-degree ramp height calculation | Ramp-height calculation | auto | subsystem4_test.c | A 100 mm ramp distance at 30 degrees produces an approximately correct height of 50 mm. |
| B4-R02 | Verify zero ramp angle handling | Ramp-height calculation | auto | subsystem4_test.c | A zero ramp angle returns an invalid result. |
| B4-R03 | Verify ramp angle above 90 degrees | Ramp-height calculation | auto | subsystem4_test.c | An angle above 90 degrees returns an invalid result. |
| B4-R04 | Verify zero ramp distance handling | Ramp-height calculation | auto | subsystem4_test.c | A zero ramp distance returns an invalid result. |
| B4-R05 | Verify ramp distance outside supported bounds | Ramp-height calculation | auto | subsystem4_test.c | A distance beyond the supported bound returns an invalid result. |
| B4-M01 | Verify physical IMU sensor detection and communication | IMU hardware and I2C communication | manual | Connect the GY-511 sensor to the Pico W, run the IMU initialisation and detection code, and check the detected accelerometer address and raw readings. | Accelerometer is detected at address `0x19` and raw accelerometer/magnetometer readings can be obtained. |
| B4-M02 | Verify physical sensor readings respond to orientation changes | IMU hardware validation | manual | Keep the sensor connected, rotate it through different orientations including a flat 360-degree rotation, and observe the raw accelerometer and magnetometer readings. | Sensor readings change when its orientation changes. |
| B4-M03 | Verify six-position accelerometer calibration | Accelerometer calibration | manual | Place the sensor in the six positive and negative axis orientations, record the X, Y and Z readings, and calculate and apply the calibration offsets and scale factors. | Calibration constants are obtained for all three axes and corrected readings are closer to the expected ±1000 mg values when stationary in each orientation. |
| B4-M04 | Verify stationary readings after calibration | Calibrated IMU readings and motion classification | manual | Keep the sensor stationary, observe calibrated X, Y and Z readings, motion activity and impact delta, and check the resulting event. | Readings remain reasonably stable, motion activity and impact delta remain low, and the event is classified as STATIONARY. |
| B4-M05 | Verify normal lift and gentle lift stop the motors | Lift detection and end-of-run signal | manual | Lift the robot normally and gently 10 times while it is stationary and while following a line. Use brisk and slow but unmistakable upward motion; restore it only after observing the stopped state. | Every normal lift changes TERRAIN to LIFTED, sets the lifted flag, and stops the motors. No false lift occurs during 60 seconds of normal stationary or driving vibration. |
| B4-M06 | Verify sideways and upside-down orientation stops the motors | Overturn detection | manual | Hold the robot upright, tilt it beyond 60 degrees for more than 200 ms, and repeat on each side and upside down 10 times. Return it upright for 500 ms. | The robot changes to OVERTURNED/FAULT-free state and stops within 200 ms. It clears after about 500 ms while upright. |
| B4-M07 | Verify hump height measurement (course hump) | Terrain hump measurement (right-wheel hump, roll based) | manual | 1. Normal build (not TEST=1), B4_HUMP_DEBUG 1. 2. Power up still on flat ground; wait for "[HUMP] level learned". 3. Line-follow over the course hump (10 mm high, 9.5 cm up, 0.5 cm flat, 9.5 cm down) with only the RIGHT wheel on it, at normal (slow) speed. 4. Repeat 5 times, including at least one run where the car stops or searches for the line on the hump. 5. Save the console log. | Each crossing prints exactly one "[B4-IMU] HUMP #n" line with height 10 +/- 1.5 mm (8.5-11.5 mm), and the dashboard shows the same value (whole mm). No HUMP line and no false count while driving on flat floor. |
| B4-M08 | Verify B4 lift peak driving baseline | Lift false-lift tuning | manual | Drive normally for 60 seconds with the dashboard/TERRAIN lift peak visible. Lift 10 times, and record peak and loop timing. | Largest 60-second driving peak is stable. Trigger is at least 1.5 times that peak. 10 normal lifts trigger and 60 seconds of driving produce no false lifts. |
| B4-M09 | Verify roll-based hump height on the bench | Terrain hump measurement (right-wheel hump) | manual | 1. Power up on flat ground and wait 1 s. 2. Put a 10 mm spacer under the RIGHT rear wheel only. 3. Read "Hump now" in the 5 s console line. 4. Repeat with 5 mm. | "Hump now" about 100 (10.0 mm) and about 50 (5.0 mm), both within 10 (1 mm). If negative, set B4_HUMP_ROLL_SIGN to -1. |
| B4-K01 | 10 mm right-wheel hump at 10 cm/s | Hump height (buddy4_hump.h) | auto | subsystem4_test.c | One hump reported, height 9.0–11.0 mm. |
| B4-K02 | Same hump with +/-150 mg vibration | Hump height (buddy4_hump.h) | auto | subsystem4_test.c | One hump reported, height 9.0–11.0 mm. |
| B4-K03 | Tilt with no right-wheel travel | Hump false-positive rejection | auto | subsystem4_test.c | No hump reported. |
| B4-K04 | Flat floor with +/-200 mg vibration | Hump false-positive rejection | auto | subsystem4_test.c | No hump reported. |
| B4-K05 | 3 mm bump | Hump minimum height | auto | subsystem4_test.c | No hump reported. |
| B4-K06 | Turn acceleration removed by lat_comp_mg | Hump turn compensation | auto | subsystem4_test.c | Reading stays level, no hump reported. |
| B4-K07 | Roll ratio to height conversion | Hump height (buddy4_hump.h) | auto | subsystem4_test.c | 90 mg at 1 g on a 111 mm track gives 10.0 mm. |
| B4-K08 | Car stops halfway up the ramp for 8 s, then continues | Hump measurement during line search | auto | subsystem4_test.c | One hump reported, height 9.0–11.0 mm. |
| B4-K09 | Reset while lifted, then put down | Hump level re-learn | auto | subsystem4_test.c | Reading is level (within 1 mm) once re-learned. |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
<!-- | YYYY-MM-DD | S4-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | | -->

| 2026-11-02 | B4-U01–U15, B4-R01–R05 | '60cc1bf' | Jie Xin | PASS | Console test output | 22 passed, 0 failed. |
| 2026-11-02 | B4-U01–U16, B4-R01–R05 | '60cc1bf' | Jie Xin | PASS | Latest Buddy 4 console test output | 23 passed, 0 failed. Includes the reserved-bit test B4-U16. |
| 2026-10-30 | B4-M01 | '14d89ec' | Jie Xin | PASS | Physical IMU detection output | LSM303DLHC accelerometer detected at I2C address `0x19`; raw readings obtained. |
| 2026-10-30 | B4-M02 | '14d89ec' | Jie Xin | PASS | Physical sensor readings | Raw accelerometer and magnetometer readings changed during flat 360-degree rotation and orientation changes. |
| 2026-10-30 | B4-M03 | '14d89ec' | Jie Xin | PASS | Six-position calibration readings | Calibration values calculated for X, Y and Z axes and applied in the IMU processing code. |
| 2026-10-30 | B4-M04 | '14d89ec' | Jie Xin | PASS | Serial output showing calibrated readings | Stationary samples showed low motion activity and impact delta, with event `0x01` and state `1` (STATIONARY). |
| 2026-11-25 | B4-M09 | ed33ebd (earlier working copy) | thrynky | PASS | logs/2026-11-25_hump_bench.txt | 10 mm block under right rear wheel: h = 93-113 (9.3-11.3 mm), roll sign correct. Pitch check read -11 mm (sign flipped since). |
| 2026-11-25 | B4-M07 | ed33ebd (earlier working copy) | thrynky | FAIL | logs/2026-11-25_hump_drive_before_fix.txt | Reported 15.5 mm (pitch check 9.9 mm) and count x5. Cause: level baseline ~3 mm off and never re-learned; 6 s timeout too short. Fixed in buddy4_hump.h (baseline follows when moving, pre-hump snapshot, 10 s / 40 cm limits). Retest needed. |