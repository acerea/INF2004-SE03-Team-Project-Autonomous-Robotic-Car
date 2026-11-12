
#ifndef BUDDY4_CLASSIFIER_H
#define BUDDY4_CLASSIFIER_H

#include <stdint.h>
#include "buddy4_data.h"

/*
 * Thresholds are provisional test values.
 * They must be validated on the physical robot.
 *
 * Units:
 *   motion_level_mg: acceleration activity metric (mg)
 *   vertical_delta_mg: Z-axis deviation from gravity baseline (mg)
 *   accel_x_mg/y_mg/z_mg: calibrated acceleration (mg)
 *   yaw_rate_mdps: yaw angular velocity (millidegrees/second)
 */

#define B4_MOTION_THRESHOLD_MG       150
#define B4_TURN_THRESHOLD_MDPS     30000
#define B4_LOW_TURN_THRESHOLD_MDPS 15000

typedef struct
{
    int16_t motion_level_mg;
    int16_t vertical_delta_mg;

    int16_t accel_x_mg;
    int16_t accel_y_mg;
    int16_t accel_z_mg;

    int16_t yaw_rate_mdps;
    uint8_t imu_fault  : 1;    /* 1 = sensor read failed            */
    uint8_t lifted     : 1;    /* 1 = handling detector: lifted     */
    uint8_t overturned : 1;    /* 1 = handling detector: overturned */
} B4_MOTION_INPUT;

/* Return absolute value safely, including INT16_MIN. */
static inline int32_t b4_abs_i16(int16_t value)
{
    int32_t wide_value = value;

    if (wide_value < 0)
    {
        wide_value = -wide_value;
    }

    return wide_value;
}

/*
 * Classify one preprocessed sensor sample.
 */
static inline B4_EVENT b4_classify_motion(B4_MOTION_INPUT input)
{
    int32_t turn_magnitude;

    turn_magnitude = b4_abs_i16(input.yaw_rate_mdps);

    uint8_t overturned = (uint8_t)input.overturned;

    /* Sensor fault takes priority. */
    if (input.imu_fault != 0u)
    {
        return b4_pack_event(B4_STATE_FAULT, 0u, 1u);
    }

    /*
     * Lifted is reported as the motion state (the robot is being
     * carried). Overturned is an orientation flag that is reported
     * with whatever motion state applies. Collision detection was
     * removed by the change of requirement.
     */
    if (input.lifted != 0u)
    {
        return b4_pack_event(B4_STATE_LIFTED, overturned, 0u);
    }

    /*
     * Turning is not physically validated.
     * The current LSM303DLHC has no gyroscope.
     */
    if (turn_magnitude >= B4_TURN_THRESHOLD_MDPS)
    {
        return b4_pack_event(B4_STATE_TURNING, overturned, 0u);
    }

    /*
     * Acceleration activity is a provisional movement indicator.
     * It cannot reliably detect constant-speed movement.
     */
    if (input.motion_level_mg >= B4_MOTION_THRESHOLD_MG)
    {
        return b4_pack_event(B4_STATE_MOVING, overturned, 0u);
    }

    /*
     * Low activity is classified as stationary provisionally.
     * Wheel encoder information should be integrated later.
     */
    if (turn_magnitude <= B4_LOW_TURN_THRESHOLD_MDPS)
    {
        return b4_pack_event(B4_STATE_STATIONARY, overturned, 0u);
    }

    return b4_pack_event(B4_STATE_UNKNOWN, overturned, 0u);
}

#endif