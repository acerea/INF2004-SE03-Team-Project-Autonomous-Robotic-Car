/** @file wire_format.h
 *
 * @brief Bit-packed frame helpers and field widths shared by all subsystems.
 *
 * @par
 * Bits are packed LSB first. Signed fields use offset-binary.
 * Header, heartbeat, command and telemetry frame widths are defined here.
 * Types 1-4 (state, motion, line, barcode) are added during integration.
 */
#ifndef WIRE_FORMAT_H
#define WIRE_FORMAT_H

#include <stdint.h>
#include <stdbool.h>

#define PROTO_VERSION           (1u)
#define PROTO_MAX_FRAME_BYTES   (16u)

// Common header, all frames: seq | version | type = 14 bits.
#define PROTO_SEQ_BITS          (8u)
#define PROTO_VERSION_BITS      (2u)
#define PROTO_TYPE_BITS         (4u)
#define PROTO_HEADER_BITS       (14u)

// Heartbeat body: link | uptime (s, wraps) | dropped-publish ticks.
#define PROTO_HB_LINK_BITS      (3u)
#define PROTO_HB_UPTIME_BITS    (16u)
#define PROTO_HB_DROP_BITS      (8u)
#define PROTO_HB_CMD_OK_BITS    (8u)    // Accepted commands, wraps at 256.
#define PROTO_HB_CMD_BAD_BITS   (8u)    // Rejected commands, wraps at 256.

// Remote command (host -> robot, no type field): seq | ver | cmd | arg.
#define PROTO_CMD_BITS          (3u)
#define PROTO_ARG_BITS          (4u)
#define PROTO_CMD_FRAME_BYTES   (3u)

typedef uint8_t proto_type_t;
#define PROTO_TYPE_HEARTBEAT    (0u)
#define PROTO_TYPE_STATE        (1u)
#define PROTO_TYPE_MOTION       (2u)
#define PROTO_TYPE_LINE         (3u)
#define PROTO_TYPE_BARCODE      (4u)
#define PROTO_TYPE_TERRAIN      (5u)
#define PROTO_TYPE_OBSTACLE     (6u)
#define PROTO_TYPE_OBST_PROFILE (7u)


// Vehicle state (type 1, Vehicle Controller): 12 bits. On change and 1 s.
#define PROTO_ST_STATE_BITS     (3u)    // 0-7, VC_STATE_xxx.
#define PROTO_ST_OBSTACLE_BITS  (2u)    // 0 clear, 1 slow down, 2 emergency.
#define PROTO_ST_LINK_LOST_BITS (1u)    // 1 = remote link lost.
#define PROTO_ST_SPEED_BITS     (6u)    // Target speed, 0-63 cm/s.

// Barcode (type 4, subsystem 3): 8 bits. Send once per barcode read.
#define PROTO_BC_LETTER_BITS    (5u)    // 0-25 = 'A'-'Z'; 31 = unreadable.
#define PROTO_BC_LETTER_BAD     (31u)
#define PROTO_BC_CMD_BITS       (3u)    // Navigation command, below.
#define PROTO_BC_CMD_NONE       (0u)
#define PROTO_BC_CMD_LEFT       (1u)
#define PROTO_BC_CMD_RIGHT      (2u)
#define PROTO_BC_CMD_STRAIGHT   (3u)
#define PROTO_BC_CMD_UTURN      (4u)

// Terrain (type 5, subsystem 4): 18 bits. Send on change and every 1 s.
#define PROTO_TER_STATE_BITS    (3u)    // 0-7, B4_STATE_xxx.
#define PROTO_TER_LIFTED_BITS   (1u)    // 1 = car lifted or overturned.
#define PROTO_TER_COLLIDE_BITS  PROTO_TER_LIFTED_BITS   // Old name, until S4 switches.
#define PROTO_TER_FAULT_BITS    (1u)    // 1 = IMU fault.
#define PROTO_TER_HUMP_OK_BITS  (1u)    // 0 = hump not measured yet.
#define PROTO_TER_HUMP_MM_BITS  (8u)    // Peak hump height, 0-255 mm.
#define PROTO_TER_HUMPS_BITS    (4u)    // Hump count, 0-15.

// Obstacle (type 6, subsystem 5): 11 bits. Send at 5 Hz.
#define PROTO_OBS_STATE_BITS    (2u)    // 0 clear, 1 slow down, 2 emergency.
#define PROTO_OBS_CM_BITS       (9u)    // Distance 1-400 cm; 0 = no echo.
#define PROTO_OBS_CM_MAX        (400u)

// Obstacle profile (type 7, subsystem 5): 27 bits. Send once per sweep.
#define PROTO_PRO_SIDE_BITS     (2u)    // 0 none, 1 left, 2 right.
#define PROTO_PRO_WIDTH_BITS    (7u)    // Obstacle width, 0-100 cm.
#define PROTO_PRO_WIDTH_MAX     (100u)
#define PROTO_PRO_LEFT_BITS     (9u)    // Left clearance, 0-400 cm.
#define PROTO_PRO_RIGHT_BITS    (9u)    // Right clearance, 0-400 cm.

typedef struct
{
    uint8_t * p_buf;
    uint32_t  cap_bits;
    uint32_t  pos_bits;
    bool      b_error;      // Set on overflow, clipping or bad width.
} proto_bits_t;

void proto_init(proto_bits_t * p_bits, uint8_t * p_buf, uint32_t cap_bytes);
void proto_begin(proto_bits_t * p_bits, proto_type_t type);
uint32_t proto_end(proto_bits_t const * p_bits);

void proto_open(proto_bits_t * p_bits, uint8_t const * p_buf,
                uint32_t len_bytes);

void proto_put(proto_bits_t * p_bits, uint32_t value, uint8_t n_bits);
void proto_put_sat(proto_bits_t * p_bits, uint32_t value, uint8_t n_bits);
void proto_put_s(proto_bits_t * p_bits, int32_t value, uint8_t n_bits);

uint32_t proto_get(proto_bits_t * p_bits, uint8_t n_bits);
int32_t proto_get_s(proto_bits_t * p_bits, uint8_t n_bits);

#endif /* WIRE_FORMAT_H */

/*** end of file ***/