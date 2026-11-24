# SE03 Fixes: changes made and cleanup list

Status as of 24 Nov 2026. Everything below is in the `SE03 Fixes` folder.

---

## 1. Things to delete

### Before handing the folder over
| What | Where | Why |
|---|---|---|
| All `_backup_before_*` folders (18) | folder root | Snapshots taken before each change. Not used by the build. |
| `logs/serial.log`, `logs/station_*.log` | `logs/` | Test-run logs. Keep the `logs/` folder itself (the station writes into it). |
| `.DS_Store` | folder root | macOS metadata. |
| `Claude outputs/` | folder root | Holds the MQTT setup guide only. Move it somewhere if you want to keep it. |
| `build_make/` build outputs | `build_make/` | Run `make clean` there instead of deleting by hand. |

### Temporary debug code (remove once the car is stable)
| What | Where |
|---|---|
| `g_ir_stage`, `IR_STAGE()` macro and every `IR_STAGE(n)` call | `app_program/subsystem3/ir_sense.c` (around lines 124-130 and in `ir_sensor_task`) |
| `ir_debug_stage()` function | `ir_sense.c` (end of file) and its prototype in `ir_sense.h` |
| `ir: DEBUG ADC ch%d read failed / first reading` prints and the `s_err_prints` / `s_ok_seen` counters | `ir_sense.c`, `ir_read_adc()` |
| `ir: DEBUG first ADC read, CS=...` print | `ir_sense.c`, start of `ir_sensor_task` |
| `ir: DEBUG tk_loc_mtx failed` print | `ir_sense.c`, `ir_sensor_task` |
| The IR-stage part of the "IR task not responding" warning | `app_program/vehicle/vehicle.c`, `vc_check_ir()`. **Keep the warning itself**; only drop the two `ir_debug_stage()` arguments and the `stage %u ch %u` text. |

---

## 2. Changes made

### Root-cause fixes
- **`lib/libtk/fastmlock.c`**: the fast lock used plain interrupt disable/enable, which is not safe on two cores. The device-manager lock got corrupted and the IR task froze inside `tk_srea_dev`. Now uses atomic operations (`atomic_inc/dec/bitset/bitclr`) when SMP is on. **This fixed the IR freeze.**
- **`device/adc/sysdepend/rp2040/adc_rp2040.c`**: ADC conversion now polls the result with bounded waits (no interrupts, FIFO off), so a read can't hang.
- **`lib/liblwip/sysdepend/pico_rp2040/include/arch/cc.h`**: added an `atoi` prototype so the build works without extra setup.

### IR sensors and calibration (`app_program/subsystem3/ir_sense.c/.h`)
- Per-sensor calibration (min/max for left, centre and right).
- Sweep calibration: records min/max while the car pans left and right (`ir_cal_sweep_begin/end/cancel`).
- ADC channels remapped to the new analog (AO) wiring: left = ADC2 (GP28), centre = ADC1 (GP27), right = ADC0 (GP26).
- Calibration result published over MQTT (`s3_publish_ircal`).
- Quieter serial output; barcode timing fix (`bar_first_ms`).

### Vehicle controller (`app_program/vehicle/vehicle.c/.h`)
- New `CALIBRATING` state; MODE 0 from the dashboard runs sweep calibration when idle or stopped.
- Calibration: sweep left 40°, sweep right, return, then centre on the line with small 6° nudges, checking after each nudge and correcting overshoot.
- Default cruise speed 3 cm/s (30 mm/s); MODE 1-15 sets cruise speed.
- Line following:
  - on the line: steer by it
  - **all three sensors black (junction or barcode): count as "line present" and go straight**. This is the junction fix. Detection itself is unchanged.
  - line just lost: go straight for up to 1.2 s before searching
  - lost longer: nudge search, then hold
- Event codes (1-18) sent to the dashboard; "IR task not responding" warning.
- Lift/overturn detection ignored while at rest; steering unlocked on state change.

### Motion (`app_program/subsystem2/`)
- `motion_pid.c`: speed PID retuned (KP 2, KI 3) with feed-forward, so 3 cm/s is actually ~3 cm/s; gentler line steering (gain 2, max 15).
- Turns: average-travel completion plus stall boost (25 % PWM, up to 50 %). *The per-wheel speed-controlled turn tried on 24 Nov was reverted; turning is back to this.*
- `subsystem2.c`: motion requests applied in order; `s2_setLineCorrection` added.

### Ultrasonic (`app_program/subsystem5/`)
- Stop and warn distance both 15 cm (only cares about objects within 15 cm ahead).
- State flags set before cleared, and only on change; sensor-fault emergency path.
- `drv_ultrasonic.c`: bounded waits and retries on the echo.
- **Servo now pans gently**: 5° steps every 25 ms, 80 ms settle (`s5_point`).

### Telemetry and dashboard
- `subsystem1/wire_format.h`, `subsystem1.h`: 4-bit cruise field; new IRCAL (8) and EVENT (9) frame types.
- `tools/station/station.py`: decodes all frame types to JSON, logs each run to `logs/`, bridges panel commands, heartbeat.
- `tools/station/control_panel.html`: SE03 ground-station UI, default speed 30 mm/s, start/stop/calibrate, event log.

### Other
- `subsystem4.c`: monitor only in test builds. `buddy4_imu.c`: fewer IMU sample prints.
- `TEAM.md`: IR pin wiring updated.

---

## 3. Build and flash
```
cd ~/Downloads/SE03\ Fixes/build_make && make NET=lwip MQTT=1 -j8 2>&1 | grep -E "error|\.uf2"
```
Flash `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2` to the RPI-RP2 drive.
