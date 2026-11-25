# Subsystem 3 — test log

**Subsystem:** Consumer (example)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | statistics self-consistent when empty or populated (min ≤ max, 0..100) | | auto | `subsystem3_test.c` | `[TEST] S3-U01 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S3-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-11-25 | S3-U02 | `f2b1748` | Kathryn | PASS | `logs/2026-11-25_unit_tests.txt` | Calibration: learned levels or safe defaults. |
| 2026-11-25 | S3-U03 | `f2b1748` | Kathryn | PASS | `logs/2026-11-25_unit_tests.txt` | Darkness normalisation 0..1000. |
| 2026-11-25 | S3-U04 | `f2b1748` | Kathryn | PASS | `logs/2026-11-25_unit_tests.txt` | Black/white hysteresis. |
| 2026-11-25 | S3-U05 | `f2b1748` | Kathryn | PASS | `logs/2026-11-25_unit_tests.txt` | Line position estimate -1000..1000. |
| 2026-11-25 | S3-U20 | `f2b1748` | Kathryn | PASS | `logs/2026-11-25_unit_tests.txt` | C/R line position excludes the barcode sensor. |
| 2026-11-25 | S3-U06 | `f2b1748` | Kathryn | PASS | `logs/2026-11-25_unit_tests.txt` | All-white reading reports line lost. |
| 2026-11-25 | S3-U07 | `f2b1748` | Kathryn | PASS | `logs/2026-11-25_unit_tests.txt` | A-Z decode at narrow = 10, 25, 40 ms. |
| 2026-11-25 | S3-U08 | `f2b1748` | Kathryn | PASS | `logs/2026-11-25_unit_tests.txt` | Decode tolerates +/-30 % element jitter. |
| 2026-11-25 | S3-U09 | `f2b1748` | Kathryn | PASS | `logs/2026-11-25_unit_tests.txt` | No wide/narrow contrast -> unreadable. |
| 2026-11-25 | S3-U10 | `f2b1748` | Kathryn | PASS | `logs/2026-11-25_unit_tests.txt` | Forward read of *A* decodes A. |
| 2026-11-25 | S3-U11 | `f2b1748` | Kathryn | PASS | `logs/2026-11-25_unit_tests.txt` | Backward read of *B* decodes B. |
| 2026-11-25 | S3-U12 | `f2b1748` | Kathryn | PASS | `logs/2026-11-25_unit_tests.txt` | Barcode word -> navigation command. |
| 2026-11-25 | S3-U13 | `f2b1748` | Kathryn | PASS | `logs/2026-11-25_unit_tests.txt` | Nav byte packing and letter codes. |
| 2026-11-25 | S3-U14 | `f2b1748` | Kathryn | PASS | `logs/2026-11-25_unit_tests.txt` | Mbf_s3_nav exists, 1-byte messages. |
| 2026-11-25 | S3-U15 | `f2b1748` | Kathryn | PASS | `logs/2026-11-25_unit_tests.txt` | Ir_sensor_task running and calibrated. |
| 2026-11-25 | S3-U16 | `f2b1748` | Kathryn | PASS | `logs/2026-11-25_unit_tests.txt` | Snapshot values within range. |
| 2026-11-25 | S3-U17 | `f2b1748` | Kathryn | PASS | `logs/2026-11-25_unit_tests.txt` | Snapshot is bit-packed into 8 bytes. |
| 2026-11-25 | S3-U18 | `f2b1748` | Kathryn | PASS | `logs/2026-11-25_unit_tests.txt` | Unframed barcode pattern is rejected. |
| 2026-11-25 | S3-U19 | `f2b1748` | Kathryn | PASS | `logs/2026-11-25_unit_tests.txt` | Invalid interior pattern rejects the word. |
