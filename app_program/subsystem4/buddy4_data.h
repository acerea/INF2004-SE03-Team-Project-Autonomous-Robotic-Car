#ifndef BUDDY4_DATA_H
#define BUDDY4_DATA_H

#include <stdint.h>

/*
 * Buddy 4 motion states.
 *
 * There are 8 possible codes (0 to 7), so the state
 * can be represented using 3 bits in a packed message.
 *
 * These are provisional software states. We can refine
 * their meaning after confirming the team's requirements.
 */
#define B4_STATE_UNKNOWN       0u
#define B4_STATE_STATIONARY    1u
#define B4_STATE_MOVING        2u
#define B4_STATE_TURNING       3u
#define B4_STATE_CLIMBING      4u
#define B4_STATE_DESCENDING    5u
#define B4_STATE_LIFTED        6u   /* Robot picked up: end of run. */
#define B4_STATE_FAULT         7u

/*
 * Packed event byte:
 *
 * Bits 0-2: motion state
 * Bit  3:   robot overturned (tilted past B4_FLIP_Z_MG)
 * Bit  4:   IMU fault detected
 *
 * Collision detection was removed (change of requirement, week 12):
 * the IMU now reports when the robot is lifted or overturned, and the
 * Vehicle Controller stops the motors on either condition.
 * Bits 5-7: reserved for future use
 */
typedef uint8_t B4_EVENT;

#define B4_STATE_MASK          0x07u
#define B4_OVERTURNED_MASK     0x08u
#define B4_IMU_FAULT_MASK      0x10u
#define B4_RESERVED_MASK       0xE0u

/* Pack the state and flags into one byte. */
static inline B4_EVENT b4_pack_event(
    uint8_t state,
    uint8_t overturned,
    uint8_t imu_fault)
{
    B4_EVENT event = 0u;

    /* Invalid state codes become UNKNOWN. */
    if (state > B4_STATE_FAULT) {
        state = B4_STATE_UNKNOWN;
    }

    event |= (B4_EVENT)(state & B4_STATE_MASK);

    if (overturned != 0u) {
        event |= B4_OVERTURNED_MASK;
    }

    if (imu_fault != 0u) {
        event |= B4_IMU_FAULT_MASK;
    }

    return event;
}

/* Extract the motion state from a packed event. */
static inline uint8_t b4_event_state(B4_EVENT event)
{
    return (uint8_t)(event & B4_STATE_MASK);
}

/* Check whether the overturned flag is set. */
static inline uint8_t b4_event_is_overturned(B4_EVENT event)
{
    return (uint8_t)((event & B4_OVERTURNED_MASK) != 0u);
}

/* Check whether the IMU fault flag is set. */
static inline uint8_t b4_event_has_imu_fault(B4_EVENT event)
{
    return (uint8_t)((event & B4_IMU_FAULT_MASK) != 0u);
}

#endif /* BUDDY4_DATA_H */