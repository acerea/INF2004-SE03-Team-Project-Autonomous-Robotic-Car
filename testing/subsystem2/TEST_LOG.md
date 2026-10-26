# Subsystem 2 — test log

**Subsystem:** Motion Control  ·  **Student ID:** 2503272  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
| S2-M01 | Basic motor movement | Motor Control API | Manual | Test forward, backward, left, right and stop | Motors respond to commands correctly |
| S2-E01 | Wheel encoder counting | FR4 | Manual | Rotate both wheels and monitor encoder counts | Counts change according to wheel rotation |
| S2-E02 | Encoder calibration | FR4 | Manual | Rotate each wheel for 5 complete revolutions | Obtain counts per revolution for both wheels |
| S2-D01 | Distance estimation | FR4 | Manual | Rotate wheels and observe calculated distance | Distance corresponds to encoder counts and calibration values |
| S2-S01 | Speed estimation | FR4 | Manual | Rotate and stop the wheels while monitoring speed | Speed changes during rotation and returns to zero when stopped |
| S2-U01 | sample values stay within 0..100 | | auto | `subsystem2_test.c` | `[TEST] S2-U01 PASS` |
| S2-U02 | free slots and free blocks within pool size | IF-02 | auto | `subsystem2_test.c` | `[TEST] S2-U02 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
| 2026-10-24 | S2-M01 | 8ec89c6 | Buddy 2 | PASS | Initial basic motor movement test |
| 2026-10-24 | S2-M01 | 8d7fc98 | Buddy 2 | PASS | Retested and updated motor movement code |
| YYYY-MM-DD | S2-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
