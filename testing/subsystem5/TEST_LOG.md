# Subsystem 5 — test log

**Subsystem:** <name>  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | example: runs in a task context | | auto | `subsystem5_test.c` | `[TEST] S5-U01 PASS` |
| S5-U02 | | | manual | 1. … 2. … | |
| S5-U11 | Profile sweep: obstacle left of the line → bypass right | FR obstacle side | auto | `subsystem5_test.c` (readings from video 25 Nov 18:25) | `[TEST] S5-U11 PASS` |
| S5-U12 | Profile sweep: obstacle right of the line → bypass left; far wall ignored | FR obstacle side | auto | `subsystem5_test.c` | `[TEST] S5-U12 PASS` |
| S5-U13 | Profile sweep: centred 10 cm obstacle, both edges found, width 8–18 cm | Rubric: measure width | auto | `subsystem5_test.c` | `[TEST] S5-U13 PASS` |
| S5-M01 | Servo direction and width on the real sensor | Rubric: measure width | manual | 1. Car on the line, box 15 cm ahead, box's right edge on the line (box on the LEFT). 2. START. 3. Read `[OBS]` lines on serial. 4. Repeat with the box centred, then on the right. | Left box: `go RIGHT`; right box: `go LEFT`; centred box: width within ±3 cm of the ruler (tune `S5_BEAM_HALF_DEG`). If left/right are swapped, set `S5_SERVO_RIGHT_SIGN` to -1. |
| S5-M02 | Bypass returns to the line | Rubric: navigate around, rejoin | manual | Box on the straight, START, watch events. | `bypass: line seen …` then `bypass done: middle sensor on the line`; car carries on along the line. |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S5-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
