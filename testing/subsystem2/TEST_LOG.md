# Subsystem 2 — test log

**Subsystem:** Motion Control  ·  **Student ID:** 2503272  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
| S2-M01 | Basic motor movement | Motor Control API | Manual | Test forward, backward, left, right and stop | Motors respond to commands correctly |
| S2-E01 | Wheel encoder counting | FR4 | Manual | Rotate both wheels and monitor encoder counts | Counts change according to wheel rotation |
| S2-E02 | Encoder calibration | FR4 | Manual | Rotate each wheel for 5 complete revolutions | Obtain counts per revolution for both wheels |
| S2-E03 | Motor-driven encoder counting | FR4 | Manual | Run both motors forward at 40% PWM for 500ms and observe encoder counts | Both encoder counts increase during motor-driven wheel rotation |
| S2-D01 | Distance estimation | FR4 | Manual | Rotate wheels and observe calculated distance | Distance corresponds to encoder counts and calibration values |
| S2-S01 | Speed estimation | FR4 | Manual | Rotate and stop the wheels while monitoring speed | Speed changes during rotation and returns to zero when stopped |
| S2-S02 | Motor-driven speed response | FR4 | Manual | Run motors at 40% and 60% PWM and compare encoder-derived speeds | Both wheels show higher estimated speed at 60% PWM |
| S2-S03 | Speed estimation with measured elapsed time | FR4 | Manual | Run motors at 40% PWM, measure elapsed time using tk_get_tim(), and calculate encoder speed | Speed calculated using actual elapsed time rather than assumed 200 ms |

| S2-U01 | sample values stay within 0..100 | | auto | `subsystem2_test.c` | `[TEST] S2-U01 PASS` |
| S2-U02 | free slots and free blocks within pool size | IF-02 | auto | `subsystem2_test.c` | `[TEST] S2-U02 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
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

| YYYY-MM-DD | S2-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
