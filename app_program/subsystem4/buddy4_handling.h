/** @file buddy4_handling.h
 *
 * @brief Lift and overturn detection from the accelerometer (Buddy 4).
 *
 * @par
 * Change of requirement (week 12): collision detection is removed. The IMU
 * instead detects when the robot has been LIFTED (picked up by hand, used
 * to signal the end of the run in the demonstration) or OVERTURNED. Either
 * condition must stop the motors at once; the Vehicle Controller does this
 * when it sees S4_FLG_LIFTED or S4_FLG_OVERTURNED in flg_s4_imu.
 *
 * @par
 * The functions here are pure (no I/O, no kernel calls), so they can be
 * unit-tested with synthetic samples in subsystem4_test.c.
 *
 * Method (LSM303DLHC has an accelerometer only, no gyroscope):
 *
 *  LIFTED     Lifting the robot accelerates it upwards, so Z (which
 *             points up) rises above its resting value. The excess
 *             z - baseline is fed into a leaky integrator (time constant
 *             B4_LIFT_TAU_MS), i.e. an upward-velocity estimate. The
 *             baseline is a slow average of z (B4_LIFT_BASE_TAU_MS), which
 *             removes calibration error. Z only, not |a|: driving
 *             vibration on X/Y always adds to |a| and built up into a
 *             false LIFTED after ~2 s of driving (2026-11-18 CSV); on Z it
 *             averages to zero. Tilting only lowers z, so it cannot lift. A hump or ledge jolt is
 *             followed by a rebound (the chassis ends at rest on the
 *             floor), so its velocity estimate returns to zero quickly; a
 *             lift keeps it positive. The trigger must hold for
 *             B4_LIFT_RUN_MS. LIFTED then latches for B4_LIFT_HOLD_MS
 *             after the last trigger. All times are in ms (dt is measured),
 *             because the IMU loop runs slower than its nominal 10 ms.
 *
 *             Host simulation (100 Hz, values below): no false trigger
 *             for 60 s of 150 mg RMS per-axis vibration, 150 humps with
 *             0.4-1 g jolts, or 50 drops off a 1 cm ledge. Lift detected
 *             after 420 ms at 0.1 g, 250 ms at 0.15 g, 150 ms at 0.3 g,
 *             120 ms at 0.5 g. Vibration of 250 mg RMS per axis or more
 *             DOES false-trigger: log lift_acc while driving (S4-M01) and
 *             keep B4_LIFT_TRIGGER at least 1.5 x its largest value.
 *
 *  OVERTURNED The Z axis points up. Its gravity share is cos(tilt), so
 *             z < B4_FLIP_Z_MG (500 mg, tilt > 60 deg) held for
 *             B4_FLIP_SET_MS means the robot is on its side or upside
 *             down. A hump never tilts the robot that far for that long.
 *             It clears with hysteresis: z > B4_FLIP_CLEAR_Z_MG for
 *             B4_FLIP_CLEAR_MS.
 *
 * All thresholds are provisional and MUST be tuned on the robot
 * (testing/subsystem4/TEST_LOG.md, S4-M01 to S4-M04).
 */
#ifndef BUDDY4_HANDLING_H
#define BUDDY4_HANDLING_H

#include <stdint.h>
#include <stdbool.h>

#define B4_ONE_G_MG (1000) // Calibrated 1 g, see processing.h.

// All times are in ms, so the detector works at the real sample rate (the
// IMU loop is slower than 10 ms: 12 single-register I2C reads per sample).

// Lift detector: Z axis only (Z points up).
#define B4_LIFT_TAU_MS (320)       // Integrator leak time constant.
#define B4_LIFT_BASE_TAU_MS (2560) // Z baseline time constant.
#define B4_LIFT_TRIGGER (20000)    // mg x ms (20 mg.s ~ 0.2 m/s up).
#define B4_LIFT_RUN_MS (80)        // Trigger held 80 ms.
#define B4_LIFT_HOLD_MS (2000)     // LIFTED latched 2 s after trigger.
#define B4_LIFT_ACC_LIMIT (200000) // Integrator clamp, mg x ms.

// Overturn detector.
#define B4_FLIP_Z_MG (500)       // cos(60 deg) = 0.5 g.
#define B4_FLIP_CLEAR_Z_MG (800) // cos(37 deg) = 0.8 g.
#define B4_FLIP_SET_MS (200)
#define B4_FLIP_CLEAR_MS (500)

#define B4_HANDLING_DT_MAX_MS (100) // Longer gaps count as 100 ms.

// Detector state.
typedef struct
{
    int32_t base_q8;       // Z baseline, mg x 256 (Q8).
    int32_t lift_acc;      // Leaky integral of Z excess, mg x ms.
    uint16_t lift_hold_ms; // LIFTED latch time left.
    uint16_t lift_run_ms;  // Time above trigger.
    uint16_t flip_run_ms;  // Set or clear run time.
    uint8_t b_lifted;
    uint8_t b_overturned;
    uint8_t b_seeded; // Baseline set from the first sample.
} B4_HANDLING;

/** Integer square root of a 32-bit value (floor). */
static inline uint32_t
b4_isqrt_u32(uint32_t value)
{
    uint32_t root = 0u;
    uint32_t bit = 1uL << 30;

    while (bit > value)
    {
        bit >>= 2;
    }

    while (0u != bit)
    {
        if (value >= (root + bit))
        {
            value -= root + bit;
            root = (root >> 1) + bit;
        }
        else
        {
            root >>= 1;
        }
        bit >>= 2;
    }

    return (root);
}

/** Magnitude of the acceleration vector, mg. */
static inline int32_t
b4_accel_magnitude_mg(int16_t x_mg, int16_t y_mg, int16_t z_mg)
{
    uint32_t sum = ((uint32_t)((int32_t)x_mg * x_mg)) + ((uint32_t)((int32_t)y_mg * y_mg)) + ((uint32_t)((int32_t)z_mg * z_mg));

    // NOTE: 3 x 32768^2 < 2^32, so the sum cannot overflow.
    return ((int32_t)b4_isqrt_u32(sum));
}

static inline void
b4_handling_init(B4_HANDLING *p_h)
{
    p_h->base_q8 = (int32_t)B4_ONE_G_MG * 256;
    p_h->lift_acc = 0;
    p_h->lift_hold_ms = 0u;
    p_h->lift_run_ms = 0u;
    p_h->flip_run_ms = 0u;
    p_h->b_lifted = 0u;
    p_h->b_overturned = 0u;
    p_h->b_seeded = 0u;
}

/** Forget the lift integrator (after a sensor fault). Latches are kept. */
static inline void
b4_handling_reset_integrator(B4_HANDLING *p_h)
{
    p_h->lift_acc = 0;
    p_h->lift_run_ms = 0u;
}

/** Add dt to a run timer without overflow. */
static inline uint16_t
b4_run_add(uint16_t run_ms, int32_t dt_ms, int32_t limit_ms)
{
    int32_t t = (int32_t)run_ms + dt_ms;

    return ((uint16_t)((t > limit_ms) ? limit_ms : t));
}

/**
 * Feed one calibrated sample (mg) taken dt_ms after the previous one.
 * Updates b_lifted and b_overturned.
 */
static inline void
b4_handling_update_dt(B4_HANDLING *p_h, int16_t x_mg, int16_t y_mg,
                      int16_t z_mg, int32_t dt_ms)
{
    int32_t excess;
    int32_t acc = p_h->lift_acc;

    (void)x_mg;
    (void)y_mg;

    if (dt_ms < 1)
    {
        dt_ms = 1;
    }
    else if (dt_ms > B4_HANDLING_DT_MAX_MS)
    {
        dt_ms = B4_HANDLING_DT_MAX_MS;
    }
    else
    {
        // In range.
    }

    // Start the baseline at the first real reading: calibrated z at rest
    // is ~1060-1090 mg on the robot, not 1000, and starting at 1000 made
    // LIFTED fire for ~2 s after every boot (2026-11-18 log).
    if (0u == p_h->b_seeded)
    {
        p_h->base_q8 = (int32_t)z_mg * 256;
        p_h->b_seeded = 1u;
    }

    // ---- Lift: Z only. Driving vibration on Z averages to zero, while
    //      |a| (all axes) always grows with X/Y vibration and made the
    //      old detector fire after ~2 s of normal driving. ----
    excess = (int32_t)z_mg - (p_h->base_q8 / 256);

    p_h->base_q8 += (((int32_t)z_mg * 256) - p_h->base_q8) * dt_ms / B4_LIFT_BASE_TAU_MS;

    acc = acc - (acc * dt_ms) / B4_LIFT_TAU_MS + excess * dt_ms;
    if (acc > B4_LIFT_ACC_LIMIT)
    {
        acc = B4_LIFT_ACC_LIMIT;
    }
    else if (acc < -B4_LIFT_ACC_LIMIT)
    {
        acc = -B4_LIFT_ACC_LIMIT;
    }
    else
    {
        // In range.
    }
    p_h->lift_acc = acc;

    if (acc >= B4_LIFT_TRIGGER)
    {
        p_h->lift_run_ms = b4_run_add(p_h->lift_run_ms, dt_ms, B4_LIFT_RUN_MS);
    }
    else
    {
        p_h->lift_run_ms = 0u;
    }

    if (p_h->lift_run_ms >= B4_LIFT_RUN_MS)
    {
        p_h->b_lifted = 1u;
        p_h->lift_hold_ms = (uint16_t)B4_LIFT_HOLD_MS;
    }
    else if (p_h->lift_hold_ms > (uint16_t)dt_ms)
    {
        p_h->lift_hold_ms = (uint16_t)(p_h->lift_hold_ms - dt_ms);
    }
    else
    {
        p_h->lift_hold_ms = 0u;
        p_h->b_lifted = 0u;
    }

    // ---- Overturn: Z-axis gravity share, with hysteresis. ----
    if (0u == p_h->b_overturned)
    {
        p_h->flip_run_ms = (z_mg < B4_FLIP_Z_MG)
                               ? b4_run_add(p_h->flip_run_ms, dt_ms, B4_FLIP_SET_MS)
                               : 0u;

        if (p_h->flip_run_ms >= B4_FLIP_SET_MS)
        {
            p_h->b_overturned = 1u;
            p_h->flip_run_ms = 0u;
        }
    }
    else
    {
        p_h->flip_run_ms = (z_mg > B4_FLIP_CLEAR_Z_MG)
                               ? b4_run_add(p_h->flip_run_ms, dt_ms, B4_FLIP_CLEAR_MS)
                               : 0u;

        if (p_h->flip_run_ms >= B4_FLIP_CLEAR_MS)
        {
            p_h->b_overturned = 0u;
            p_h->flip_run_ms = 0u;
        }
    }

    return;
}

/** Same, for samples exactly 10 ms apart (unit tests). */
static inline void
b4_handling_update(B4_HANDLING *p_h, int16_t x_mg, int16_t y_mg,
                   int16_t z_mg)
{
    b4_handling_update_dt(p_h, x_mg, y_mg, z_mg, 10);
}

#endif /* BUDDY4_HANDLING_H */

/*** end of file ***/
