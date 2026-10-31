
#ifndef BUDDY4_IMU_PROCESSING_H
#define BUDDY4_IMU_PROCESSING_H

#include <stdint.h>
#include "buddy4_imu.h"

/*
 * LSM303DLHC:
 * +/-2g, high-resolution mode.
 * 1 mg per digit after dividing by 16.
 */

#define B4_ACCEL_RAW_SHIFT 4

/*
 * Accelerometer calibration values obtained
 * from the six-position calibration test.
 *
 * Units:
 * Offset = mg
 * Scale  = mg per 1g
 */
#define B4_ACCEL_OFFSET_X_MG (-37.25f)
#define B4_ACCEL_OFFSET_Y_MG (17.30f)
#define B4_ACCEL_OFFSET_Z_MG (-23.25f)

#define B4_ACCEL_SCALE_X_MG (993.15f)
#define B4_ACCEL_SCALE_Y_MG (1012.10f)
#define B4_ACCEL_SCALE_Z_MG (1027.85f)

typedef struct {
    int16_t x_mg;
    int16_t y_mg;
    int16_t z_mg;
} B4_ACCEL_MG;

/* Convert one raw accelerometer axis into milligravity (mg). */
static inline int16_t b4_accel_raw_to_mg(int16_t raw)
{
    return (int16_t)(raw / (1 << B4_ACCEL_RAW_SHIFT));
}

/* Convert all three raw accelerometer axes into mg. */
static inline void b4_accel_convert(
    const B4_IMU_RAW *raw,
    B4_ACCEL_MG *converted)
{
    converted->x_mg = b4_accel_raw_to_mg(raw->accel_x);
    converted->y_mg = b4_accel_raw_to_mg(raw->accel_y);
    converted->z_mg = b4_accel_raw_to_mg(raw->accel_z);
}

/*
 * Apply offset and scale calibration to one axis.
 *
 * Corrected value is returned in mg.
 */
static inline int16_t b4_accel_calibrate_axis(
    int16_t raw_mg,
    float offset_mg,
    float scale_mg)
{
    float corrected;

    corrected = ((float)raw_mg - offset_mg)
                / scale_mg * 1000.0f;

    return (int16_t)(corrected >= 0.0f
                     ? corrected + 0.5f
                     : corrected - 0.5f);
}

/* Apply calibration to all three axes. */
static inline void b4_accel_calibrate(
    const B4_ACCEL_MG *raw_mg,
    B4_ACCEL_MG *calibrated)
{
    calibrated->x_mg = b4_accel_calibrate_axis(
        raw_mg->x_mg,
        B4_ACCEL_OFFSET_X_MG,
        B4_ACCEL_SCALE_X_MG);

    calibrated->y_mg = b4_accel_calibrate_axis(
        raw_mg->y_mg,
        B4_ACCEL_OFFSET_Y_MG,
        B4_ACCEL_SCALE_Y_MG);

    calibrated->z_mg = b4_accel_calibrate_axis(
        raw_mg->z_mg,
        B4_ACCEL_OFFSET_Z_MG,
        B4_ACCEL_SCALE_Z_MG);
}

#endif
