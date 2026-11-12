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
 *  LIFTED     Lifting the robot accelerates it upwards, so the magnitude
 *             of the measured acceleration rises above its resting value.
 *             The excess |a| - baseline (tilt-independent) is fed into a
 *             leaky integrator (time constant 2^B4_LIFT_LEAK_SHIFT
 *             samples), i.e. an upward-velocity estimate. The baseline is
 *             a slow average of |a| (2^B4_LIFT_BASE_SHIFT samples), which
 *             removes calibration error and the small positive bias that
 *             sideways vibration adds to |a|. A hump or ledge jolt is
 *             followed by a rebound (the chassis ends at rest on the
 *             floor), so its velocity estimate returns to zero quickly; a
 *             lift keeps it positive. The trigger must hold for
 *             B4_LIFT_RUN_SAMPLES samples. LIFTED then latches for
 *             B4_LIFT_HOLD_SAMPLES after the last trigger.
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
 *             B4_FLIP_SET_SAMPLES means the robot is on its side or upside
 *             down. A hump never tilts the robot that far for that long.
 *             It clears with hysteresis: z > B4_FLIP_CLEAR_Z_MG for
 *             B4_FLIP_CLEAR_SAMPLES.
 *
 * All thresholds are provisional and MUST be tuned on the robot
 * (testing/subsystem4/TEST_LOG.md, S4-M01 to S4-M04).
 */
#ifndef BUDDY4_HANDLING_H
#define BUDDY4_HANDLING_H

#include <stdint.h>
#include <stdbool.h>

#define B4_ONE_G_MG             (1000)    // Calibrated 1 g, see processing.h.

// Lift detector (10 ms samples).
#define B4_LIFT_LEAK_SHIFT      (5u)      // Integrator time constant 32 samples.
#define B4_LIFT_BASE_SHIFT      (8u)      // Baseline time constant 2.56 s.
#define B4_LIFT_TRIGGER         (2000)    // mg x samples (~0.2 m/s upwards).
#define B4_LIFT_RUN_SAMPLES     (8u)      // Trigger held 80 ms.
#define B4_LIFT_HOLD_SAMPLES    (200u)    // LIFTED latched 2 s after trigger.
#define B4_LIFT_ACC_LIMIT       (20000)   // Integrator clamp, mg x samples.

// Overturn detector.
#define B4_FLIP_Z_MG            (500)     // cos(60 deg) = 0.5 g.
#define B4_FLIP_CLEAR_Z_MG      (800)     // cos(37 deg) = 0.8 g.
#define B4_FLIP_SET_SAMPLES     (20u)     // 200 ms.
#define B4_FLIP_CLEAR_SAMPLES   (50u)     // 500 ms.

// Detector state: counters sized to their range (change requirement 1).
typedef struct
{
    int32_t  base_q8;               // |a| baseline, mg x 256 (Q8).
    int16_t  lift_acc;              // Leaky integral, clamped to +/-20000.
    uint8_t  lift_hold;             // 0..200 samples of LIFTED latch left.
    uint8_t  lift_run    : 4;       // 0..8 samples above trigger.
    uint8_t  b_lifted    : 1;
    uint8_t  b_overturned: 1;
    uint8_t  flip_run    : 6;       // 0..50 samples (set or clear run).
} B4_HANDLING;

/** Integer square root of a 32-bit value (floor). */
static inline uint32_t
b4_isqrt_u32 (uint32_t value)
{
    uint32_t root = 0u;
    uint32_t bit  = 1uL << 30;

    while (bit > value)
    {
        bit >>= 2;
    }

    while (0u != bit)
    {
        if (value >= (root + bit))
        {
            value -= root + bit;
            root   = (root >> 1) + bit;
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
b4_accel_magnitude_mg (int16_t x_mg, int16_t y_mg, int16_t z_mg)
{
    uint32_t sum = ((uint32_t) ((int32_t) x_mg * x_mg))
                 + ((uint32_t) ((int32_t) y_mg * y_mg))
                 + ((uint32_t) ((int32_t) z_mg * z_mg));

    // NOTE: 3 x 32768^2 < 2^32, so the sum cannot overflow.
    return ((int32_t) b4_isqrt_u32(sum));
}

static inline void
b4_handling_init (B4_HANDLING * p_h)
{
    p_h->base_q8      = (int32_t) B4_ONE_G_MG * 256;
    p_h->lift_acc     = 0;
    p_h->lift_hold    = 0u;
    p_h->lift_run     = 0u;
    p_h->b_lifted     = 0u;
    p_h->b_overturned = 0u;
    p_h->flip_run     = 0u;
}

/** Forget the lift integrator (after a sensor fault). Latches are kept. */
static inline void
b4_handling_reset_integrator (B4_HANDLING * p_h)
{
    p_h->lift_acc = 0;
    p_h->lift_run = 0u;
}

/**
 * Feed one calibrated sample (mg). Updates b_lifted and b_overturned.
 */
static inline void
b4_handling_update (B4_HANDLING * p_h, int16_t x_mg, int16_t y_mg,
                    int16_t z_mg)
{
    int32_t mag    = b4_accel_magnitude_mg(x_mg, y_mg, z_mg);
    int32_t excess = mag - (p_h->base_q8 / 256);
    int32_t acc    = (int32_t) p_h->lift_acc;

    // ---- Baseline: slow average of |a| (Q8 fixed point). ----
    p_h->base_q8 += ((mag * 256) - p_h->base_q8)
                    / (int32_t) (1u << B4_LIFT_BASE_SHIFT);

    // ---- Lift: leaky integrator of the excess over the baseline. ----
    acc = acc - (acc / (int32_t) (1u << B4_LIFT_LEAK_SHIFT)) + excess;
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
    p_h->lift_acc = (int16_t) acc;

    if (acc >= B4_LIFT_TRIGGER)
    {
        if (p_h->lift_run < B4_LIFT_RUN_SAMPLES)
        {
            p_h->lift_run++;
        }
    }
    else
    {
        p_h->lift_run = 0u;
    }

    if (p_h->lift_run >= B4_LIFT_RUN_SAMPLES)
    {
        p_h->b_lifted  = 1u;
        p_h->lift_hold = (uint8_t) B4_LIFT_HOLD_SAMPLES;
    }
    else if (p_h->lift_hold > 0u)
    {
        p_h->lift_hold--;
    }
    else
    {
        p_h->b_lifted = 0u;
    }

    // ---- Overturn: Z-axis gravity share, with hysteresis. ----
    if (0u == p_h->b_overturned)
    {
        p_h->flip_run = (z_mg < B4_FLIP_Z_MG) ? (p_h->flip_run + 1u) : 0u;

        if (p_h->flip_run >= B4_FLIP_SET_SAMPLES)
        {
            p_h->b_overturned = 1u;
            p_h->flip_run     = 0u;
        }
    }
    else
    {
        p_h->flip_run = (z_mg > B4_FLIP_CLEAR_Z_MG) ? (p_h->flip_run + 1u)
                                                     : 0u;

        if (p_h->flip_run >= B4_FLIP_CLEAR_SAMPLES)
        {
            p_h->b_overturned = 0u;
            p_h->flip_run     = 0u;
        }
    }

    return;
}

#endif /* BUDDY4_HANDLING_H */

/*** end of file ***/