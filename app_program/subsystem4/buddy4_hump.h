#ifndef BUDDY4_HUMP_H
#define BUDDY4_HUMP_H

#include <stdint.h>

/*
 * Buddy 4 hump detector and height estimator (accelerometer only).
 *
 * Course hump (measured 25 Nov): 10 mm high, 9.5 cm up-slope, 0.5 cm flat
 * top, 9.5 cm down-slope (about 6 deg). It sits under the RIGHT drive
 * wheel only, so the chassis mainly ROLLS (leans left) on it.
 *
 * Car geometry: two rear drive wheels B4_HUMP_TRACK_MM apart, front
 * caster about B4_HUMP_CASTER_MM ahead of the rear axle. The car rests on
 * three points, so with the right wheel raised by h:
 *
 *   roll  : tan(roll)  = h / TRACK         -> h = TRACK  * a_x / a_z
 *   pitch : tan(pitch) = h / (2 * CASTER)  -> h = 2 * CASTER * a_y / a_z
 *
 * Sensor frame (see buddy4_imu.c): X = right, Y = rear, Z = up. With the
 * right side raised, +X points slightly upwards, so a_x becomes positive;
 * the rear axle rises relative to the caster, so a_y becomes positive.
 * If the bench test reads negative, set B4_HUMP_ROLL_SIGN to -1.
 *
 * The roll estimate is used for the result. The pitch estimate is only
 * reported as a cross-check: forward acceleration also changes a_y.
 *
 * The LSM303DLHC has no gyroscope. The caller removes the sideways
 * acceleration caused by turning (v * omega, from the wheel encoders)
 * and passes it as lat_comp_mg.
 *
 * Pure integer code with no kernel calls, so it can be unit tested.
 */

/* ---- Car and course geometry ---------------------------------------- */
#define B4_HUMP_TRACK_MM        111     /* Rear tyre centre to centre.   */
#define B4_HUMP_CASTER_MM       100     /* Rear axle to caster contact.  */
#define B4_HUMP_ROLL_SIGN       1       /* Bench 25 Nov: +10 mm OK.      */
#define B4_HUMP_PITCH_SIGN      (-1)    /* Bench 25 Nov: Y read -11 mm.  */

/* ---- Detector tuning (heights in 0.1 mm, times in ms) ---------------- */
#define B4_HUMP_AVG_N           8u      /* Moving average; the loop runs
                                           every 20-40 ms, so ~240 ms.
                                           Drive slowly (<= ~10 cm/s).   */
#define B4_HUMP_START_DMM       50      /* 5.0 mm (~2.6 deg roll) to start. */
#define B4_HUMP_START_MS        100     /* ...held this long.            */
#define B4_HUMP_END_DMM         20      /* Below 2.0 mm counts as flat.  */
#define B4_HUMP_QUIET_MS        150     /* Flat this long ends the hump. */
#define B4_HUMP_MAX_MS          10000   /* Longer: not a hump, discard.
                                           On-hump part takes ~3.5 s at
                                           3.5 cm/s, plus ~1 s per line
                                           search stop. 6 s was too tight. */
#define B4_HUMP_MAX_TRAVEL_MM   400     /* Right wheel rolled further
                                           than this: not a hump.        */
#define B4_HUMP_MIN_PEAK_DMM    60      /* Ignore peaks below 6.0 mm.    */
#define B4_HUMP_MIN_TRAVEL_MM   30      /* Right wheel must move 3 cm.   */
#define B4_HUMP_MIN_Z_MG        500     /* Below this: lifted / tipped.  */

/* Why the last hump was discarded (for the serial monitor). */
#define B4_HUMP_REJ_NONE        0u
#define B4_HUMP_REJ_LOW_PEAK    1u      /* Peak below MIN_PEAK.          */
#define B4_HUMP_REJ_NO_TRAVEL   2u      /* Right wheel moved too little. */
#define B4_HUMP_REJ_TIMEOUT     3u      /* Tilted too long / too far.    */

/* ---- Level baseline (mounting tilt, floor slope) --------------------- */
#define B4_HUMP_BASE_INIT       50u     /* Samples averaged at start-up. */
#define B4_HUMP_BASE_SHIFT      6       /* Then follow at 1/64 a sample
                                           (~2 s), whenever no hump is
                                           starting or in progress.      */

/*
 * Baseline snapshots: one every B4_HUMP_HIST_EVERY samples, B4_HUMP_HIST_N
 * kept (~2.5-3.5 s at 20-40 ms a sample). When a hump starts, the height
 * is measured from the oldest one, i.e. the level before the car reached
 * the ramp; the live baseline has already crept up the first few mm.
 */
#define B4_HUMP_HIST_N          6u
#define B4_HUMP_HIST_EVERY      16u

typedef struct {
    /* Level baseline, mg x 256. */
    int32_t base_x_q8;
    int32_t base_y_q8;
    uint16_t base_count;
    int32_t hist_x_q8[B4_HUMP_HIST_N];
    int32_t hist_y_q8[B4_HUMP_HIST_N];
    uint8_t hist_idx;
    uint8_t hist_tick;

    /* Moving-average window. */
    int16_t win_x[B4_HUMP_AVG_N];
    int16_t win_y[B4_HUMP_AVG_N];
    int16_t win_z[B4_HUMP_AVG_N];
    int32_t sum_x;
    int32_t sum_y;
    int32_t sum_z;
    uint8_t win_idx;
    uint8_t win_fill;

    /* Event tracking. */
    uint8_t active;
    int32_t confirm_ms;
    int32_t active_ms;
    int32_t quiet_ms;
    int32_t start_travel_mm;
    int32_t peak_dmm;
    int32_t peak_pitch_dmm;

    /*
     * Set by the caller before b4_hump_update(): 1 while the car spins or
     * pivots (line search). Jerky turning shakes the sensor sideways, so
     * those samples do not update the peak or end the hump.
     */
    uint8_t hold;
    uint8_t hold_left;          /* Samples still ignored after a spin. */
    int32_t prev_travel_mm;     /* To tell moving from standing.       */

    /* Live values, for the console (0.1 mm). */
    int32_t now_dmm;
    int32_t now_pitch_dmm;
    int32_t now_dev_x_mg;       /* Averaged X minus level baseline.  */
    int32_t now_dev_y_mg;       /* Averaged Y minus level baseline.  */
    uint8_t ready;              /* 1 = level baseline learned.       */
    uint8_t started;            /* 1 = a hump started this sample.   */
    uint8_t rejected;           /* B4_HUMP_REJ_xxx, set when it ends. */
    int32_t reject_peak_dmm;
    int32_t reject_travel_mm;

    /* Last completed hump (valid after b4_hump_update returns 1). */
    int32_t result_dmm;
    int32_t result_pitch_dmm;
    int32_t result_travel_mm;
} B4_HUMP;

static inline void b4_hump_init(B4_HUMP *h)
{
    uint8_t i;

    h->base_x_q8 = 0;
    h->base_y_q8 = 0;
    h->base_count = 0u;
    for (i = 0u; i < B4_HUMP_HIST_N; i++)
    {
        h->hist_x_q8[i] = 0;
        h->hist_y_q8[i] = 0;
    }
    h->hist_idx = 0u;
    h->hist_tick = 0u;

    for (i = 0u; i < B4_HUMP_AVG_N; i++)
    {
        h->win_x[i] = 0;
        h->win_y[i] = 0;
        h->win_z[i] = 0;
    }
    h->sum_x = 0;
    h->sum_y = 0;
    h->sum_z = 0;
    h->win_idx = 0u;
    h->win_fill = 0u;

    h->active = 0u;
    h->confirm_ms = 0;
    h->active_ms = 0;
    h->quiet_ms = 0;
    h->start_travel_mm = 0;
    h->peak_dmm = 0;
    h->peak_pitch_dmm = 0;

    h->hold = 0u;
    h->hold_left = 0u;
    h->prev_travel_mm = 0;
    h->now_dmm = 0;
    h->now_pitch_dmm = 0;
    h->now_dev_x_mg = 0;
    h->now_dev_y_mg = 0;
    h->ready = 0u;
    h->started = 0u;
    h->rejected = B4_HUMP_REJ_NONE;
    h->reject_peak_dmm = 0;
    h->reject_travel_mm = 0;

    h->result_dmm = 0;
    h->result_pitch_dmm = 0;
    h->result_travel_mm = 0;
}

/* Forget a partly seen hump (e.g. after a sensor read error). */
static inline void b4_hump_abort(B4_HUMP *h)
{
    h->active = 0u;
    h->confirm_ms = 0;
    h->active_ms = 0;
    h->quiet_ms = 0;
    h->peak_dmm = 0;
    h->peak_pitch_dmm = 0;
    h->win_fill = 0u;
    h->win_idx = 0u;
    h->sum_x = 0;
    h->sum_y = 0;
    h->sum_z = 0;
}

/* Height in 0.1 mm for a lever of lever_mm and tilt ratio dev_mg / z_mg. */
static inline int32_t b4_hump_height_dmm(int32_t lever_mm, int32_t dev_mg,
                                         int32_t z_mg)
{
    int32_t num = lever_mm * 10 * dev_mg;

    if (z_mg <= 0)
    {
        return 0;
    }
    /* Round to nearest. */
    return (num >= 0) ? ((num + (z_mg / 2)) / z_mg)
                      : ((num - (z_mg / 2)) / z_mg);
}

/*
 * Feed one calibrated sample.
 *
 *   x_mg, y_mg, z_mg : calibrated acceleration (X right, Y rear, Z up)
 *   lat_comp_mg      : sideways acceleration to remove, in the X-right
 *                      frame: +v*omega for a left (CCW) turn. 0 if unknown.
 *   travel_mm        : right-wheel distance (any origin, may be signed)
 *   dt_ms            : time since the previous sample
 *
 * Returns 1 when a hump has just been measured (see result_* fields).
 */
static inline uint8_t b4_hump_update(B4_HUMP *h,
                                     int16_t x_mg, int16_t y_mg,
                                     int16_t z_mg, int16_t lat_comp_mg,
                                     int32_t travel_mm, int32_t dt_ms)
{
    int32_t x_c;
    int32_t avg_x;
    int32_t avg_y;
    int32_t avg_z;
    int32_t dev_x;
    int32_t dev_y;
    int32_t height;
    int32_t pitch;
    int32_t moved;
    uint8_t moving;
    uint8_t done = 0u;

    h->started = 0u;
    h->rejected = B4_HUMP_REJ_NONE;

    if (dt_ms < 0)
    {
        dt_ms = 0;
    }

    /* Lifted, tipped or upside down: the handling detector owns this. */
    if (z_mg < B4_HUMP_MIN_Z_MG)
    {
        b4_hump_abort(h);
        h->now_dmm = 0;
        h->now_pitch_dmm = 0;
        return 0u;
    }

    /*
     * Moving = the right wheel turned since the last sample. The level
     * baseline and the peak only change while moving: a car standing
     * on the ramp (line search) must not teach the baseline that the
     * slope is level, and must not get extra chances at a noisy peak.
     */
    moving = (travel_mm != h->prev_travel_mm) ? 1u : 0u;
    h->prev_travel_mm = travel_mm;

    /* After a spin, also skip until the average window has flushed. */
    if (h->hold != 0u)
    {
        h->hold_left = (uint8_t)B4_HUMP_AVG_N;
    }
    else if (h->hold_left > 0u)
    {
        h->hold_left--;
    }

    /* Remove the turning acceleration (a left turn reads as -X). */
    x_c = (int32_t)x_mg + (int32_t)lat_comp_mg;

    /* Moving average over the last B4_HUMP_AVG_N samples. */
    h->sum_x += x_c - h->win_x[h->win_idx];
    h->sum_y += (int32_t)y_mg - h->win_y[h->win_idx];
    h->sum_z += (int32_t)z_mg - h->win_z[h->win_idx];
    h->win_x[h->win_idx] = (int16_t)x_c;
    h->win_y[h->win_idx] = y_mg;
    h->win_z[h->win_idx] = z_mg;
    h->win_idx = (uint8_t)((h->win_idx + 1u) % B4_HUMP_AVG_N);

    if (h->win_fill < B4_HUMP_AVG_N)
    {
        /* Window not full yet: the unfilled slots are zero. */
        h->win_fill++;
        if (h->win_fill < B4_HUMP_AVG_N)
        {
            return 0u;
        }
    }

    avg_x = h->sum_x / (int32_t)B4_HUMP_AVG_N;
    avg_y = h->sum_y / (int32_t)B4_HUMP_AVG_N;
    avg_z = h->sum_z / (int32_t)B4_HUMP_AVG_N;

    /* Start-up: learn the level baseline (car must be on flat ground). */
    if (h->base_count < B4_HUMP_BASE_INIT)
    {
        h->base_count++;
        h->base_x_q8 += ((avg_x * 256) - h->base_x_q8)
                        / (int32_t)h->base_count;
        h->base_y_q8 += ((avg_y * 256) - h->base_y_q8)
                        / (int32_t)h->base_count;
        if (h->base_count >= B4_HUMP_BASE_INIT)
        {
            uint8_t k;

            for (k = 0u; k < B4_HUMP_HIST_N; k++)
            {
                h->hist_x_q8[k] = h->base_x_q8;
                h->hist_y_q8[k] = h->base_y_q8;
            }
            h->ready = 1u;
        }
        return 0u;
    }

    dev_x = avg_x - (h->base_x_q8 / 256);
    dev_y = avg_y - (h->base_y_q8 / 256);

    height = B4_HUMP_ROLL_SIGN
           * b4_hump_height_dmm(B4_HUMP_TRACK_MM, dev_x, avg_z);
    pitch  = B4_HUMP_PITCH_SIGN
           * b4_hump_height_dmm(2 * B4_HUMP_CASTER_MM, dev_y, avg_z);

    h->now_dmm = height;
    h->now_pitch_dmm = pitch;
    h->now_dev_x_mg = dev_x;
    h->now_dev_y_mg = dev_y;

    if (h->active == 0u)
    {
        if (height >= B4_HUMP_START_DMM)
        {
            h->confirm_ms += dt_ms;
            if (h->confirm_ms >= B4_HUMP_START_MS)
            {
                /* Measure from the level before the ramp (oldest
                   snapshot; hist_idx is the next one to overwrite). */
                h->base_x_q8 = h->hist_x_q8[h->hist_idx];
                h->base_y_q8 = h->hist_y_q8[h->hist_idx];
                dev_x = avg_x - (h->base_x_q8 / 256);
                dev_y = avg_y - (h->base_y_q8 / 256);
                height = B4_HUMP_ROLL_SIGN
                       * b4_hump_height_dmm(B4_HUMP_TRACK_MM, dev_x, avg_z);
                pitch  = B4_HUMP_PITCH_SIGN
                       * b4_hump_height_dmm(2 * B4_HUMP_CASTER_MM, dev_y,
                                            avg_z);
                h->now_dmm = height;
                h->now_pitch_dmm = pitch;

                h->active = 1u;
                h->active_ms = h->confirm_ms;
                h->quiet_ms = 0;
                h->start_travel_mm = travel_mm;
                h->peak_dmm = height;
                h->peak_pitch_dmm = pitch;
                h->started = 1u;
            }
        }
        else
        {
            h->confirm_ms = 0;

            if (moving == 0u)
            {
                return 0u;
            }

            /*
             * No hump: let the baseline follow slow changes (floor
             * slope, a wrong start-up level). Test 25 Nov: a 3 mm
             * start-up error never recovered when this only ran below
             * END_DMM, and it added 3 mm to the hump.
             */
            h->base_x_q8 += ((avg_x * 256) - h->base_x_q8)
                            >> B4_HUMP_BASE_SHIFT;
            h->base_y_q8 += ((avg_y * 256) - h->base_y_q8)
                            >> B4_HUMP_BASE_SHIFT;

            h->hist_tick++;
            if (h->hist_tick >= B4_HUMP_HIST_EVERY)
            {
                h->hist_tick = 0u;
                h->hist_x_q8[h->hist_idx] = h->base_x_q8;
                h->hist_y_q8[h->hist_idx] = h->base_y_q8;
                h->hist_idx = (uint8_t)((h->hist_idx + 1u) % B4_HUMP_HIST_N);
            }
        }
        return 0u;
    }

    /* Hump in progress. */
    h->active_ms += dt_ms;

    if ((h->hold != 0u) || (h->hold_left > 0u))
    {
        /* Spinning: ignore this sample, but keep the hump open. */
        if (h->active_ms > B4_HUMP_MAX_MS)
        {
            h->rejected = B4_HUMP_REJ_TIMEOUT;
            h->reject_peak_dmm = h->peak_dmm;
            h->reject_travel_mm = 0;
            h->active = 0u;
            h->confirm_ms = 0;
            h->active_ms = 0;
            h->quiet_ms = 0;
            h->peak_dmm = 0;
            h->peak_pitch_dmm = 0;
        }
        return 0u;
    }

    if ((moving != 0u) && (height > h->peak_dmm))
    {
        h->peak_dmm = height;
        h->peak_pitch_dmm = pitch;
    }

    /* Off the hump: below END_DMM, or below a third of the peak (a
       start-up level error of a few mm must not keep it open). */
    if ((height < B4_HUMP_END_DMM) || (height < (h->peak_dmm / 3)))
    {
        h->quiet_ms += dt_ms;
    }
    else
    {
        h->quiet_ms = 0;
    }

    if (h->quiet_ms >= B4_HUMP_QUIET_MS)
    {
        moved = travel_mm - h->start_travel_mm;
        if (moved < 0)
        {
            moved = -moved;
        }

        if ((h->peak_dmm >= B4_HUMP_MIN_PEAK_DMM) &&
            (moved >= B4_HUMP_MIN_TRAVEL_MM))
        {
            h->result_dmm = h->peak_dmm;
            h->result_pitch_dmm = h->peak_pitch_dmm;
            h->result_travel_mm = moved;
            done = 1u;
        }
        else
        {
            h->rejected = (h->peak_dmm < B4_HUMP_MIN_PEAK_DMM)
                        ? B4_HUMP_REJ_LOW_PEAK : B4_HUMP_REJ_NO_TRAVEL;
            h->reject_peak_dmm = h->peak_dmm;
            h->reject_travel_mm = moved;
        }

        h->active = 0u;
        h->confirm_ms = 0;
        h->active_ms = 0;
        h->quiet_ms = 0;
        h->peak_dmm = 0;
        h->peak_pitch_dmm = 0;
    }
    else if ((h->active_ms > B4_HUMP_MAX_MS) ||
             (((travel_mm - h->start_travel_mm) > B4_HUMP_MAX_TRAVEL_MM) ||
              ((h->start_travel_mm - travel_mm) > B4_HUMP_MAX_TRAVEL_MM)))
    {
        /* Tilted for too long (parked on a slope, held at an angle). */
        h->rejected = B4_HUMP_REJ_TIMEOUT;
        h->reject_peak_dmm = h->peak_dmm;
        h->reject_travel_mm = 0;
        h->active = 0u;
        h->confirm_ms = 0;
        h->active_ms = 0;
        h->quiet_ms = 0;
        h->peak_dmm = 0;
        h->peak_pitch_dmm = 0;
    }

    return done;
}

#endif /* BUDDY4_HUMP_H */
