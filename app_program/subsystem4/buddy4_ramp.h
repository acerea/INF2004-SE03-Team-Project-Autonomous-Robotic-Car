#ifndef BUDDY4_RAMP_H
#define BUDDY4_RAMP_H

#include <stdint.h>

/*
 * Buddy 4 ramp-height estimator (software-only first stage).
 *
 * Inputs:
 *   distance_mm: distance travelled along the ramp, in millimetres.
 *   angle_deci_deg: ramp angle in tenths of a degree (e.g. 150 = 15.0 deg).
 *
 * Output:
 *   estimated vertical height in millimetres.
 *
 * IMPORTANT:
 * This module does NOT read the IMU or measure distance. The caller must
 * supply validated angle/distance estimates. Sensor acquisition, angle
 * estimation, distance source, and real-ramp calibration remain to be done.
 */

#define B4_RAMP_MAX_DISTANCE_MM 10000u
#define B4_RAMP_MAX_ANGLE_DECI_DEG 900u
#define B4_RAMP_INVALID_HEIGHT 0xFFFFu

typedef struct {
    uint16_t height_mm;
    uint8_t valid;
} B4_RAMP_RESULT;

/*
 * Approximate sin(angle) with the Bhaskara-I formula using integer math.
 * Returns sin(angle) scaled by 1000, where 1000 represents 1.000.
 * Angle must be in tenths of a degree, from 0 to 90 degrees.
 */
static inline uint16_t b4_ramp_sin_milli(uint16_t angle_deci_deg)
{
    uint32_t x;
    uint32_t product;
    uint32_t numerator;
    uint32_t denominator;

    if (angle_deci_deg > B4_RAMP_MAX_ANGLE_DECI_DEG) {
        return 0u;
    }

    /* Formula scaled by 10: sin(x) ~= 4*x*(1800-x)/(4050000-x*(1800-x)). */
    x = angle_deci_deg;
    product = x * (1800u - x);
    numerator = 4u * product * 1000u;
    denominator = 4050000u - product;

    if (denominator == 0u) {
        return 0u;
    }

    return (uint16_t)(numerator / denominator);
}

/*
 * Estimate vertical height from ramp distance and angle.
 * Returns valid=0 and B4_RAMP_INVALID_HEIGHT for invalid inputs.
 */
static inline B4_RAMP_RESULT b4_estimate_ramp_height(
    uint16_t distance_mm,
    uint16_t angle_deci_deg)
{
    B4_RAMP_RESULT result;
    uint32_t sin_milli;
    uint32_t height;

    result.height_mm = B4_RAMP_INVALID_HEIGHT;
    result.valid = 0u;

    if (distance_mm == 0u || distance_mm > B4_RAMP_MAX_DISTANCE_MM ||
        angle_deci_deg == 0u ||
        angle_deci_deg > B4_RAMP_MAX_ANGLE_DECI_DEG) {
        return result;
    }

    sin_milli = b4_ramp_sin_milli(angle_deci_deg);
    height = ((uint32_t)distance_mm * sin_milli + 500u) / 1000u;

    if (height > B4_RAMP_MAX_DISTANCE_MM) {
        return result;
    }

    result.height_mm = (uint16_t)height;
    result.valid = 1u;
    return result;
}

#endif /* BUDDY4_RAMP_H */
