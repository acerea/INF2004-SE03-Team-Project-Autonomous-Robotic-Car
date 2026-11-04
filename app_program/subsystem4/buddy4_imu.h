#ifndef BUDDY4_IMU_H
#define BUDDY4_IMU_H

#include "../app.h"
#include <stdint.h>

/*
 * GY-511 breakout, normally based on the ST LSM303DLHC.
 * Accelerometer: 0x18 or 0x19 (selected by SA0).
 * Magnetometer:  0x1E.
 *
 * This is a raw acquisition/diagnostic stage. It does not yet feed the
 * provisional classifier or calculate ramp height.
 */
#define B4_IMU_ACCEL_ADDR_DEFAULT 0x19u
#define B4_IMU_ACCEL_ADDR_ALT     0x18u
#define B4_IMU_MAG_ADDR           0x1Eu

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
} B4_IMU_RAW;

IMPORT void b4_imu_task(INT stacd, void *exinf);

#endif /* BUDDY4_IMU_H */