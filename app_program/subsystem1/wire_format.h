/** @file wire_format.h
 *
 * @brief Bit-packed frame helpers and field widths shared by all subsystems.
 *
 * @par
 * Bits are packed LSB first. Signed fields use offset-binary.
 * Header, heartbeat and command widths are defined here. Telemetry frame
 * layouts (types 1-6) are added by each owner during integration.
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