
#ifndef BUDDY4_IMU_PROCESSING_H
#define BUDDY4_IMU_PROCESSING_H

#include <stdint.h>
#include "buddy4_imu.h"

/*
 * LSM303DLHC:
 * +/-2g, high-resolution mode.
 * 1 mg per digit after shifting right by 4 bits.
 */

#define B4_ACCEL_RAW_SHIFT 4

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

/* Convert all three accelerometer axes. */
static inline void b4_accel_convert(
    const B4_IMU_RAW *raw,
    B4_ACCEL_MG *converted)
{
    converted->x_mg = b4_accel_raw_to_mg(raw->accel_x);
    converted->y_mg = b4_accel_raw_to_mg(raw->accel_y);
    converted->z_mg = b4_accel_raw_to_mg(raw->accel_z);
}

#endif
