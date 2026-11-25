# Videos

Short videos of tests and demos running on the real hardware. A video shows
what a log cannot: the LED blinking, the motor turning, the robot reaching
the line. List **every** video in the table at the bottom of this page.

## What to record

- **Short:** 15–60 seconds. One test or one feature per video.
- **Say what it is.** Show or say the test ID (e.g. `S2-U03`, `INT-01`) or
  the demo name at the start.
- **Show the commit.** Before recording, run `git rev-parse --short HEAD` and
  keep the terminal in shot for a moment, or put it in the table below.
- **Show the result.** Keep both the hardware and, if it matters, the
  console output in view.
- **Landscape**, steady, good light. No music needed.

## How to share it: unlisted YouTube only

1. Upload the video to YouTube with visibility set to **Unlisted**: anyone
   with the link can watch it, but it does not show up in search. Not
   *Private* (only invited accounts can watch, so the link won't work for
   your team or instructor), and not *Public*.
2. Copy the video's link (e.g. `https://youtu.be/...`).

**Do not commit video files to this repo or attach them on GitHub.** A video
in the repo stays in the git history for good, even if deleted later, and
slows every clone for the whole team. Video files are ignored by
`.gitignore` for this reason.

Then add a row to the table below. If the video is evidence for a test, also
put its link in the *Log / evidence* column of that test's row in
`testing/.../TEST_LOG.md`.

## Video index

Newest at the bottom. Never delete a row.

| Date | Subsystem | Test ID / demo | Commit | Student ID | Link | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S1 | S1-U03 | `abc1234` | | <https://youtu.be/...> | LED blinks at 2 Hz |
| 2026-11-25 | INT | INT run R1 (INT-VC5, VC6, VC7, VC9, VC11) | `f2b1748` | 2502049 | <https://youtu.be/KNKkJJi9sS0> | Line following, hump 11 mm, obstacle sweep -> bypass right, back on the line. Logs: `testing/integration/logs/2026-11-25_R1_*.log` |
| 2026-11-25 | INT | INT run R2 (INT-VC5, VC6, VC7, VC9, VC11) | `f2b1748` | 2502049 | <https://youtu.be/FVoq1ijGTDE> | Line following, hump 11 mm, obstacle sweep -> bypass right, back on the line. Logs: `testing/integration/logs/2026-11-25_R2_*.log` |
| 2026-11-25 | INT | INT run R3 (INT-VC6, VC7, VC9, VC10, VC11) | `f2b1748` | 2502049 | <https://youtu.be/xRuwYrglJrI> | Hump 11 mm, bypass, back on the line, picked up -> motors stop. Logs: `testing/integration/logs/2026-11-25_R3_*.log` |
