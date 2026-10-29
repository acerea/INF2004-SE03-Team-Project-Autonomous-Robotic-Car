# Subsystem 2 — test log

**Subsystem:** Motion Control  ·  **Student ID:** 2503272  ·  **Code:** `app_program/subsystem2/`, `app_program/subsystem2/subsystem2_test.c`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | Sample value range | Existing subsystem test | auto | Run `subsystem2_test.c` | `[TEST] S2-U01 PASS` |
| S2-U02 | Semaphore and memory pool bounds | IF-02 | auto | Run `subsystem2_test.c` | `[TEST] S2-U02 PASS` |
| S2-M01 | Basic motor movement | Motor Control API | manual | Test forward, backward, left, right and stop | Motors respond to commands correctly |
| S2-E01 | Manual encoder counting | FR4 | manual | Rotate both wheels and observe encoder counts | Counts change according to wheel rotation |
| S2-E02 | Preliminary CPR calibration | FR4 | manual | Rotate each wheel for five complete revolutions | Obtain preliminary counts per revolution (CPR) for both wheels |
| S2-E03 | Motor-driven encoder | FR4 | manual | Run both motors at 40% PWM for 500 ms | Both encoder counts increase during motor-driven rotation |
| S2-S01 | Speed estimation using measured time | FR4 | manual | Use tk_get_tim() to measure the sampling interval | Speed calculations use measured elapsed time |
| S2-S02 | Motor-driven speed response | FR4 | manual | Compare encoder-derived speeds at 40% and 60% PWM | Estimated wheel speeds increase with PWM |
| S2-S03 | Manual speed estimation | FR4 | manual | Rotate and stop wheels while monitoring speed | Speed responds to rotation and returns to zero when stationary |
| S2-ED01 | Motor-driven encoder distance | FR4 | manual | Compare encoder distance estimates with measuring-tape measurements | Record actual and encoder-estimated distances, calculate percentage error for both wheels, and identify whether further calibration is required. |
| S2-M02 | Independent wheel PWM control | FR5 | manual | Set left PWM to 40% and right PWM to 60%, then stop both motors after 2 seconds | Both motors operate independently at their assigned PWM levels and stop |
| S2-P01 | P controller PWM calculation | FR5 | auto | Call `speed_p_update(15.0f, 12.0f, 40.0f, 2.0f)` and check the calculated PWM | Output is 46% PWM and test prints PASS |
| S2-P02 | P control with encoder feedback | FR5 | manual | Run both motors with a target speed of 15 cm/s, measure encoder speeds and adjust each wheel's PWM | PWM adjustments respond in the correct direction to speed errors |
| S2-P03 | PI controller calculation | FR5 | manual | Call PI update twice with target=15 cm/s, measured=12 cm/s, dt=0.2 s | PWM increases from 46.6% to 47.2%, demonstrating accumulated error |
| S2-P04 | PI control with encoder feedback | FR5 | manual | Run both motors with a target speed of 15 cm/s and adjust each wheel's PWM using independent PI controllers | PWM responds to speed error and accumulated error; assess speed stability |
| S2-P05 | Full PID calculation | FR5 | manual | Run three PID updates with changing speed errors and measured dt=0.2 s | Calculated PWM outputs are 46.6%, 44.5% and 42.7% |
| S2-P06 | Full PID with encoder feedback | FR5 | manual | Run both motors with a 15 cm/s target and adjust PWM using independent PID controllers | Both wheel PWM values respond to encoder speed errors; assess speed stability |
| S2-P07 | Integrated PID wheel speed control | FR5 | manual | Run 10 PID feedback cycles at 15 cm/s using measured elapsed time | PID executes successfully and responds to encoder feedback |
| S2-P08 | PID control loop timing | NFR2 | manual | Run 20 PID updates and measure elapsed time using `tk_get_tim()` | All 20 updates complete with intervals ≤20 ms |
| S2-P09 | Straight-line motion correction | FR5 | manual | Compare wheel-speed differences with STRAIGHT_KP=0.5 and 0.0; conduct physical straight-line trials with STRAIGHT_KP=0.0 and 0.1 | Wheel speeds remain similar and physical sideways drift is reduced |
| S2-D01 | Encoder-based distance movement | FR6 | manual | Command 20 cm and 30 cm movements; compare encoder and actual distances | Robot stops at the requested encoder distance; measure physical distance error |
| S2-T01 | Encoder-based turning | FR6 | manual | Test 90° left, 90° right and 180° right turns; compare encoder travel against calculated targets | Turns complete when both wheels reach their target distances; assess turning accuracy |


*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-24 | S2-M01 | 8ec89c6 | Buddy 2 | PASS | Initial basic motor movement test |
| 2026-10-24 | S2-M01 | 8d7fc98 | Buddy 2 | PASS | Retested and updated motor movement code |
| 2026-10-26 | S2-M01 | `30b53d5` | Ying Jing | PASS | `logs/2026-10-26_M01.png` | Movement and stopping verified |
| 2026-10-26 | S2-E01 | `30b53d5` | Ying Jing | PASS | `logs/2026-10-26_E01.png` | Left and right encoder counts change when wheels rotate | 
| 2026-10-26 | S2-E02 | `30b53d5` | Ying Jing | PASS | `logs/2026-10-26_E02.png` | At 5 revolutions for each wheel: L=12567, Left CPR=2513 counts; R=12666, Right CPR= 2533 counts; CPR calibration is preliminary |
| 2026-10-26 | S2-E03 | `30b53d5` | Ying Jing | PASS | `logs/2026-10-26_E03.png` | Motor-driven counting at 40% PWM for 500 ms (Left +1492, Right +1517) |
| 2026-10-26 | S2-S01 | `30b53d5` | Ying Jing | PASS | `logs/2026-10-26_S01.png` | Elapsed=210 ms; L=15.7 cm/s, R=16.6 cm/s; Absolute speed accuracy not verified |
| 2026-10-26 | S2-S02 | `30b53d5` | Ying Jing | PASS | `logs/2026-10-26_S02.png` | At 40% PWM (L=16.0cm/s, R=17.4cm/s); At 60% PWM (L=23.4cm/s, R=24.2cm/s); Absolute speed accuracy not verified |
| 2026-10-26 | S2-S03 | `30b53d5` | Ying Jing | PASS | `logs/2026-10-26_S03.png` | Speed changes during wheel rotation and returns to zero when stopped |
| 2026-10-27 | S2-ED01 | `30b53d5` | Ying Jing | PASS | `logs/2026-10-27_ED01.png` | Actual distance 12.7 cm; left encoder 12.2 cm (3.94% error), right encoder 12.3 cm (3.15% error). Mean error 3.54%; Further calibration pending. |
| 2026-10-27 | S2-M02 | `02fea4b` | Ying Jing | PASS | `logs/2026-10-27_M02.png` | Left PWM=40%, right PWM=60%; both motors stopped after 2 seconds |
| 2026-10-27 | S2-P01 | `02fea4b` | Ying Jing | PASS | `logs/2026-10-27_P01_1.png` | Target=15 cm/s, measured=12 cm/s, base PWM=40%, Kp=2; calculated PWM=46% |
| 2026-10-27 | S2-P02 | `02fea4b` | Ying Jing | PASS | `logs/2026-10-27_P02.png` | PWM adjusted independently based on encoder speed; L=9–18 cm/s, R=18–20 cm/s. Target-speed stability not yet achieved |
| 2026-10-27 | S2-P03 | `02fea4b` | Ying Jing | PASS | `logs/2026-10-27_P03.png` | PI calculation verified: PWM increased from 46.6% to 47.2% due to accumulated speed error |
| 2026-10-27 | S2-P04 | `02fea4b` | Ying Jing | PASS | `logs/2026-10-27_P04_1.png` | PI adjusts left and right PWM independently; speeds approach 17–18 cm/s but remain above the 15 cm/s target. Further tuning required |
| 2026-10-27 | S2-P04 | `02fea4b` | Ying Jing | PASS | `logs/2026-10-27_P04_2.png` | PI feedback adjusts PWM using encoder speed; L=13–20 cm/s, R=14–19 cm/s against a 15 cm/s target. Speed stability and accuracy require further tuning |
| 2026-10-27 | S2-P05 | `02fea4b` | Ying Jing | FAIL | `logs/2026-10-27_P05_1.png` | PID calculated PWM=46.6%, 44.5%, 42.7%, but the test failed due to incorrect expected values in the test code |
| 2026-10-27 | S2-P05 | `02fea4b` | Ying Jing | PASS | `logs/2026-10-27_P05_2.png` | Corrected expected values; PID calculation verified with PWM=46.6%, 44.5%, 42.7% |
| 2026-10-27 | S2-P06 | `02fea4b` | Ying Jing | FAIL | `logs/2026-10-27_P06_1.png` | Full PID feedback executed, but large speed oscillations occurred (L=6–27 cm/s, R=6–30 cm/s); PWM repeatedly reached 0%. PID tuning required |
| 2026-10-27 | S2-P06 | `02fea4b` | Ying Jing | FAIL | `logs/2026-10-27_P06_2.png` | Retested with Kp=2, Ki=1, Kd=0.01, base PWM=30%. Speeds fluctuated and dropped to L=1, R=0 cm/s in cycle 10 despite PWM increasing to 61–62%. Further investigation required |
| 2026-10-27 | S2-P06 | `02fea4b` | Ying Jing | PASS | `logs/2026-10-27_P06_3.png` | Full PID feedback with Kp=2, Ki=1, Kd=0.01, base PWM=30%. Target=15 cm/s; cycles 5–10: L=15–17 cm/s, R=15–16 cm/s. Basic speed tracking demonstrated; formal accuracy and 20 ms timing validation pending |
| 2026-10-28 | S2-P07 | `737a812` | Ying Jing | PASS | `logs/2026-10-28_P07.png` | Integrated PID completed 10 cycles at 210 ms intervals. L=9–21 cm/s, R=12–20 cm/s; Speed stability and ≤20 ms timing still pending |
| 2026-10-28 | S2-P08 | `737a812` | Ying Jing | FAIL | `logs/2026-10-28_P08_1.png` | 20/20 cycles completed; min=max=avg=30 ms using 19 ms delay; exceeded NFR2 |
| 2026-10-28 | S2-P08 | `737a812` | Ying Jing | PASS | `logs/2026-10-28_P08_2.png` | 20/20 cycles completed; min=max=avg=20 ms using 10 ms delay; further timing validation under load pending |
| 2026-10-28 | S2-P09 | `737a812` | Ying Jing | FAIL | `logs/2026-10-28_P09_1.png` | STRAIGHT_KP=0.5; wheel-speed difference increased to 12 cm/s by cycle 20. Correction unstable; further tuning required |
| 2026-10-28 | S2-P09 | `737a812` | Ying Jing | PASS | `logs/2026-10-28_P09_2.png` | STRAIGHT_KP=0.0; wheel-speed difference remained within 0–2 cm/s over 20 cycles. Independent PID gave more stable wheel-speed matching; physical straight-line accuracy not yet verified |
| 2026-10-28 | S2-P10 | `737a812` | Ying Jing | PASS | `2026-10-28_setup.png` | KP=0.0; forward=72.5, 73.0, 72.8 cm; drift=4 cm right, 2.3 cm left, no noticeable drift. Baseline recorded |
| 2026-10-28 | S2-P10 | `737a812` | Ying Jing | PASS | `2026-10-28_setup.png` | KP=0.1; forward=73.0, 72.9, 73.0 cm; no noticeable drift across 3 trials. Improved straight-line accuracy compared with baseline |
| 2026-10-28 | S2-D01 | `737a812` | Ying Jing | PASS | `2026-10-28_setup.png`, `2026-10-28_D01_1.png` | Target=30 cm; actual≈33 cm; overshoot≈3 cm (10%). Basic stopping works, accuracy needs improvement |
| 2026-10-28 | S2-D01 | `737a812` | Ying Jing | PASS | `2026-10-28_setup.png`, `2026-10-28_D01_2.png` | Target=30 cm; encoder L=30.61 cm, R=30.82 cm; actual≈33 cm. Overshoot remains |
| 2026-10-28 | S2-D01 | `737a812` | Ying Jing | PASS | `2026-10-28_setup.png`, `2026-10-28_D01_3.png` | Target=20 cm; encoder L=21.02 cm, R=20.46 cm; actual=20.5 cm (2.5% error) |
| 2026-10-28 | S2-T01 | `63b7be3` | Ying Jing | PASS | `logs/2026-10-28_T01_1.png` | Initial 90° left-turn test completed (Completed=1). Physical angle appeared slightly slanted; encoder distances not recorded yet |
| 2026-10-28 | S2-T01 | `63b7be3` | Ying Jing | PASS | `logs/2026-10-28_T01_2.png` | 90° left: L=-80 mm, R=83 mm; independent wheel stopping improved accuracy |
| 2026-10-28 | S2-T01 | `63b7be3` | Ying Jing | PASS | `logs/2026-10-28_T01_3.png` | 90° right: L=81 mm, R=-79 mm; expected ±77.8 mm |
| 2026-10-28 | S2-T01 | `63b7be3` | Ying Jing | PASS | `logs/2026-10-28_T01_4.png` | 180° right: L=155 mm, R=-158 mm; expected ±155.5 mm |
| 2026-10-28 | S2-T01 | `63b7be3` | Ying Jing | PASS | `logs/2026-10-28_T01_5.png` | 180° left turn completed; L=-157 mm, R=159 mm against expected ±155.5 mm |