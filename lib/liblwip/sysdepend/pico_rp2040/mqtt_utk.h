/** @file mqtt_utk.h
 *
 * @brief MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1).
 *
 * @par
 * Plain C only: no lwIP, SDK or kernel headers, so app_program/ may include
 * it. Subsystem 1 talks to this library through two message buffers.
 */
#ifndef MQTT_UTK_H
#define MQTT_UTK_H

#include <stdint.h>

// Queued TX message = 1 flag byte + 1 frame. Must match subsystem1.h.
#define MQTT_UTK_TX_MAX_MSG     (17u)
#define MQTT_UTK_TX_FLAG_ACK    (0x01u)     // Publish this frame with QoS 1.
#define MQTT_UTK_RX_MAX_MSG     (4u)        // Largest command accepted.
#define MQTT_UTK_WILL_MAX       (16u)       // Largest Last-Will frame.

// Where the frame type sits in a frame (see wire_format.c, proto_begin).
// Heartbeat frames go to the heartbeat topic, everything else to telemetry.
#define MQTT_UTK_TYPE_BYTE      (1u)
#define MQTT_UTK_TYPE_SHIFT     (2u)
#define MQTT_UTK_TYPE_MASK      (0x0Fu)
#define MQTT_UTK_TYPE_HEARTBEAT (0u)

/** Give the library its message buffers and Last-Will frame. Call once from
 *  subsystem1_init(). The library does not connect until this has run.
 *  The will frame must contain no zero bytes (lwIP uses strlen on it). */
void mqtt_utk_bind(int32_t mbf_tx_id, int32_t mbf_rx_id,
                   uint8_t const * p_will, uint32_t will_len);

/** Called by cyw43_utk_task() once the link is up. Safe to call repeatedly. */
void mqtt_utk_start(void);

/** Time (kernel ms) of the last proof of broker contact, 0 if never. */
uint32_t mqtt_utk_last_contact_ms(void);

/** Frames the library had to drop (publish refused or bad size). */
uint32_t mqtt_utk_tx_dropped(void);

/** Commands dropped because the command message buffer was full. */
uint32_t mqtt_utk_rx_dropped(void);

#endif /* MQTT_UTK_H */

/*** end of file ***/