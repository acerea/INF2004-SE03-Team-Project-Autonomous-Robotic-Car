#ifndef BUDDY4_CLASSIFIER_H
#define BUDDY4_CLASSIFIER_H

#include <stdint.h>
#include "buddy4_data.h"

/*
 * Stage 2 prototype classifier.
 *
 * These thresholds are provisional test values, not validated
 * thresholds for the physical robot.
 *
 * Units:
 *   motion_level_mg: windowed motion/activity metric in milli-g
 *   vertical_delta_mg: vertical acceleration deviation in milli-g
 *   yaw_rate_mdps: yaw angular velocity in milli-degrees/second
 *
 * The IMU driver or preprocessing stage must convert sensor readings
 * into these units before calling this classifier.
 */

#define B4_MOTION_THRESHOLD_MG       150
#define B4_HUMP_THRESHOLD_MG         700
#define B4_IMPACT_THRESHOLD_MG      1800
#define B4_TURN_THRESHOLD_MDPS     30000
#define B4_LOW_TURN_THRESHOLD_MDPS 15000

typedef struct
{
    int16_t motion_level_mg;
    int16_t vertical_delta_mg;
    int16_t yaw_rate_mdps;
    uint8_t imu_fault;
} B4_MOTION_INPUT;

/* Return the absolute value safely, including for INT16_MIN. */
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
 * Classify one preprocessed sample.
 *
 * Note: real-world classification will require filtering and
 * multiple samples over time to avoid reacting to sensor noise.
 */
static inline B4_EVENT b4_classify_motion(B4_MOTION_INPUT input)
{
    int32_t vertical_magnitude;
    int32_t turn_magnitude;

    vertical_magnitude = b4_abs_i16(input.vertical_delta_mg);
    turn_magnitude = b4_abs_i16(input.yaw_rate_mdps);

    /* A sensor fault takes priority over normal classification. */
    if (input.imu_fault != 0u)
    {
        return b4_pack_event(B4_STATE_FAULT, 0u, 1u);
    }

    /* A large acceleration deviation is treated as a possible impact. */
    if (vertical_magnitude >= B4_IMPACT_THRESHOLD_MG)
    {
        return b4_pack_event(B4_STATE_IMPACT, 1u, 0u);
    }

    /* A sufficiently large yaw rate indicates a possible turn. */
    if (turn_magnitude >= B4_TURN_THRESHOLD_MDPS)
    {
        return b4_pack_event(B4_STATE_TURNING, 0u, 0u);
    }

    /* Detect a hump-like vertical acceleration pattern. */
    if (vertical_magnitude >= B4_HUMP_THRESHOLD_MG)
    {
        if (input.vertical_delta_mg > 0)
        {
            return b4_pack_event(B4_STATE_CLIMBING, 0u, 0u);
        }

        return b4_pack_event(B4_STATE_DESCENDING, 0u, 0u);
    }

    /*
     * This uses a windowed activity metric rather than one
     * instantaneous acceleration sample.
     */
    if (input.motion_level_mg >= B4_MOTION_THRESHOLD_MG)
    {
        return b4_pack_event(B4_STATE_MOVING, 0u, 0u);
    }

    /* Avoid declaring stationary if the gyro reading is ambiguous. */
    if (turn_magnitude <= B4_LOW_TURN_THRESHOLD_MDPS)
    {
        return b4_pack_event(B4_STATE_STATIONARY, 0u, 0u);
    }

    return b4_pack_event(B4_STATE_UNKNOWN, 0u, 0u);
}

#endif