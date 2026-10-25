/** @file subsystem1.h
 *
 * @brief Subsystem 1 (Comms): public interface.
 *
 * @par
 * Student ID: 2502049. Wire format: see the project report.
 */
#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include <stdint.h>
#include <stdbool.h>
#include "../app.h"
#include "wire_format.h"

#define S1_PRI_COMMS        (22)      // Lowest app priority
#define S1_TICK_MS          (100u)    // Comms housekeeping period.
#define S1_HEARTBEAT_MS     (2000u)   // NFR4: at least every 2 s.
#define S1_LINK_STALE_MS    (5000u)   // NFR6: no broker contact = "lost".
#define S1_LINK_FAULT_MS    (10000u)  // NFR6: not restored in 10 s = FAULT.

// Queued telemetry message = one flag byte followed by one frame.
#define S1_TLM_FLAG_ACK     (0x01u)   // Publish with QoS 1.
#define S1_TLM_MAX_MSG      (1u + PROTO_MAX_FRAME_BYTES)
#define S1_TLM_BUF_BYTES    (256u)
#define S1_RAW_MAX_MSG      (4u)
#define S1_RAW_BUF_BYTES    (32u)
#define S1_CMD_MAX_MSG      (1u)
#define S1_CMD_BUF_BYTES    (32u)
#define S1_CMD_FRAME_BYTES  PROTO_CMD_FRAME_BYTES

// Bits of g_flg_remote, waited on by the Vehicle Controller.
#define S1_FLG_ESTOP        (0x01u)
#define S1_FLG_STOP         (0x02u)
#define S1_FLG_LINK_LOST    (0x04u)   // Set whenever not CONNECTED.
#define S1_FLG_CMD_READY    (0x08u)   // A command is in g_mbf_remote_cmd.

// Bit of g_flg_comms, private to subsystem 1.
#define S1_FLG_TLM_DROP     (0x01u)

// Bits of g_s1_fault_bits. Written only by comms_task.
#define S1_FAULT_WIFI_LOST      (0x01u)
#define S1_FAULT_MQTT_LOST      (0x02u)
#define S1_FAULT_CMD_OVERFLOW   (0x04u)
#define S1_FAULT_TLM_DROP       (0x08u)

// Commands. Wire: seq (8) | ver (2) | cmd (3) | arg (4).
#define S1_CMD_START        (0u)
#define S1_CMD_STOP         (1u)
#define S1_CMD_ESTOP        (2u)
#define S1_CMD_RESET        (3u)
#define S1_CMD_MODE         (4u)
#define S1_CMD_BITS         PROTO_CMD_BITS
#define S1_ARG_BITS         PROTO_ARG_BITS

// Link state, 3 bits on the wire (8 values), one byte in RAM.
typedef uint8_t s1_link_t;
#define S1_LINK_INIT            (0u)
#define S1_LINK_WIFI_CONNECTING (1u)
#define S1_LINK_MQTT_CONNECTING (2u)
#define S1_LINK_CONNECTED       (3u)
#define S1_LINK_LOST            (4u)
#define S1_LINK_RECONNECTING    (5u)
#define S1_LINK_FAULT           (6u)
#define S1_LINK_OFFLINE         (7u)    // Used only by the Last-Will frame.

typedef struct
{
    uint8_t last_seq;
    bool    b_have_seq;
} s1_seq_state_t;

// Remote command as queued for the Vehicle Controller: one byte,
// cmd in bits 0-2 and arg in bits 3-6.
static inline uint8_t
s1_cmd_pack (uint32_t cmd, uint32_t arg)
{
    // NOTE: cmd < 8 and arg < 16, so the result fits in 7 bits.
    return ((uint8_t) ((arg << S1_CMD_BITS) | cmd));
}

static inline uint32_t
s1_cmd_of (uint8_t packed)
{
    return (((uint32_t) packed) & ((1u << S1_CMD_BITS) - 1u));
}

static inline uint32_t
s1_arg_of (uint8_t packed)
{
    return ((((uint32_t) packed) >> S1_CMD_BITS) & ((1u << S1_ARG_BITS) - 1u));
}

// Kernel objects (IDs published for the other subsystems).
IMPORT ID g_mbf_telemetry;      // IF-06: all -> S1 (use s1_publish).
IMPORT ID g_mbf_remote_cmd;     // IF-07: S1 -> Vehicle Controller.
IMPORT ID g_flg_remote;         // IF-07: bits S1_FLG_xxx.
IMPORT volatile uint8_t g_s1_fault_bits;    // IF-08: S1 -> Vehicle Controller.

// Private to subsystem 1.
IMPORT ID g_mbf_cmd_raw;
IMPORT ID g_flg_comms;
IMPORT void comms_task(INT stacd, void * exinf);

/** Queue one frame for MQTT. Task context only, never blocks. Returns E_OK
 *  or a negative error (E_TMOUT means the queue was full: frame dropped). */
ER s1_publish(uint8_t const * p_frame, uint32_t n_bytes, bool b_ack);

/** Validate a raw command frame. True if accepted; *p_packed is set. */
bool s1_cmd_check(uint8_t const * p_raw, uint32_t len,
                  s1_seq_state_t * p_seq, uint8_t * p_packed);

#endif /* SUBSYSTEM1_H */

/*** end of file ***/