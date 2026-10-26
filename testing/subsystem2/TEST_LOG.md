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
| S2-E01 | Encoder counting | FR4 | manual | Rotate both wheels and observe encoder counts | Counts change according to wheel rotation |
| S2-E02 | Encoder calibration | FR4 | manual | Rotate each wheel for five complete revolutions | Obtain preliminary counts per revolution (CPR) for both wheels |
| S2-E03 | Motor-driven encoder counting | FR4 | manual | Run both motors at 40% PWM for 500 ms | Both encoder counts increase during motor-driven rotation |
| S2-D01 | Distance estimation | FR4 | manual | Manually rotate wheels and observe calculated distance | Encoder counts are converted into wheel distance |
| S2-D02 | Motor-driven distance validation | FR4 | manual | Compare encoder distance estimates with measuring-tape measurements | Record distance error and assess consistency |
| S2-S01 | Speed estimation using measured time | FR4 | manual | Use tk_get_tim() to measure the sampling interval | Speed calculations use measured elapsed time |
| S2-S02 | Motor-driven speed response | FR4 | manual | Compare encoder-derived speeds at 40% and 60% PWM | Estimated wheel speeds increase with PWM |
| S2-S03 | Speed estimation | FR4 | manual | Rotate and stop wheels while monitoring speed | Speed responds to rotation and returns to zero when stationary |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

### Validation Results (26 October 2026)
| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-26 | S2-M01 | `8d7fc98` | Ying Jing | PASS | `logs/2026-10-26_motorMovement.png` | Movement and stopping verified |
| 2026-10-26 | S2-E01 | `9edc79c` | Ying Jing | PASS | `logs/2026-10-26_manualEncoderCounts.png` | Left and right encoder counts change when wheels rotate | 
| 2026-10-26 | S2-E02 | `9edc79c` | Ying Jing | PASS | `logs/2026-10-26_encoderCalibration.png` | At 5 revolutions for each wheel: L=12567, Left CPR=2513 counts; R=12666, Right CPR= 2533 counts |
| 2026-10-26 | S2-E03 | `9edc79c` | Ying Jing | PASS | `logs/2026-10-26_motorDrivenEncoder.png` | Motor-driven counting at 40% PWM for 500 ms (Left +1492, Right +1517) |
| 2026-10-26 | S2-D01 | `9edc79c` | Ying Jing | PASS | `logs/2026-10-26_encoderBasedDistance.png` | Distance calculated from encoder counts and wheel calibration |
| 2026-10-26 | S2-S01 | `9edc79c` | Ying Jing | PASS | `logs/2026-10-26_elapsedTimeSpeed.png` | Elapsed=210 ms; L=15.7 cm/s, R=16.6 cm/s; absolute speed accuracy not yet verified |
| 2026-10-26 | S2-S02 | `9edc79c` | Ying Jing | PASS | `logs/2026-10-26_speedResponse.png` | At 40% PWM (L=16.0cm/s, R=17.4cm/s); At 60% PWM (L=23.4cm/s, R=24.2cm/s) |
| 2026-10-26 | S2-S03 | `9edc79c` | Ying Jing | PASS | `logs/2026-10-26_manualSpeed.png` | Speed changes during wheel rotation and returns to zero when stopped |


### Validation Results (24–25 October 2026)
| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-24 | S2-M01 (Initial) | `8ec89c6` | Buddy 2 | PASS | Manual Observation | Basic motor movement verified |
| 2026-10-25 | S2-M01 (Retest) | `8d7fc98` | Buddy 2 | PASS | Manual observation | Movement and stopping verified |
| 2026-10-25 | S2-E01 | COMMIT_HASH | Buddy 2 | PASS | Manual serial output | Left and right encoder counts change when wheels rotate |
| 2026-10-25 | S2-E02 | COMMIT_HASH | Buddy 2 | PASS | Manual serial output | Motor-driven counting at 40% PWM for 500 ms (Left +1200, Right +1178). Accuracy validation pending |
| 2026-10-25 | S2-D01 | COMMIT_HASH | Buddy 2 | PASS | Manual serial output | Distance calculated from encoder counts and wheel calibration |
| 2026-10-25 | S2-S01 | COMMIT_HASH | Buddy 2 | PASS | Manual serial output | Speed changes during wheel rotation and returns to zero when stopped |
| 2026-10-25 | S2-ED02 | COMMIT_HASH | Buddy 2 | PASS | Manual serial output and measuring tape | Actual 4.4-4.5 cm; encoder L=4.2 cm, R=4.3 cm; average error ~4.5% |
| 2026-10-25 | S2-ED02 | COMMIT_HASH | Buddy 2 | PASS | Manual serial output and measuring tape | Actual 4.2 cm; encoder L=3.9 cm, R=4.4 cm; average error ~1.2% |
| 2026-10-25 | S2-S02 | COMMIT_HASH | Buddy 2 | PASS | Manual serial output | At 40% PWM (L=16.4cm/s, R=16.8cm/s); At 60% PWM (L=22.6cm/s, R=24.2cm/s). Timing and absolute accuracy pending |
| 2026-10-25 | S2-S03 | COMMIT_HASH | Buddy 2 | PASS | Manual serial output | Elapsed=210 ms; L=15.7 cm/s, R=16.6 cm/s; absolute speed accuracy not yet verified |




