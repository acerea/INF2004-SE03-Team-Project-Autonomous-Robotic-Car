/** @file mqtt_utk.c
 *
 * @brief MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1).
 *
 * @par
 * Runs entirely on cyw43_utk_task, like the rest of lwIP. Subsystem 1 never
 * calls lwIP: it fills the TX message buffer with frames and reads commands
 * from the RX message buffer. This file must not include tk/tkernel.h (size_t
 * clash, see cyw43_utk.c), so the two kernel calls it needs are declared here.
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/sys.h"
#include "lwip/timeouts.h"
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include <tm/tmonitor.h>

extern int32_t tk_snd_mbf(int32_t mbfid, void const * p_msg, int32_t msgsz,
                          int32_t tmout);
extern int32_t tk_rcv_mbf(int32_t mbfid, void * p_msg, int32_t tmout);

#ifndef MQTT_TOPIC_BASE
#define MQTT_TOPIC_BASE         MQTT_CLIENT_ID
#endif
#define TOPIC_TLM               (MQTT_TOPIC_BASE "/tlm")
#define TOPIC_HB                (MQTT_TOPIC_BASE "/hb")
#define TOPIC_CMD               (MQTT_TOPIC_BASE "/cmd")

#define KEEPALIVE_S             (10u)   // Broker drops us after ~1.5x this.
#define RETRY_MS                (3000u)
#define TX_POLL_MS              (50u)
#define TX_BURST                (8u)    // Frames published per poll.
#define FLUSH_MAX               (64u)   // Bound on discards in one go.
#define CMD_QOS                 (1u)
#define WILL_QOS                (1u)
#define TMO_POLL                (0)     // T-Kernel TMO_POL: never block.
#define SEQ_STREAMS             (2u)    // Telemetry and heartbeat.
#define STREAM_TLM              (0u)
#define STREAM_HB               (1u)

#define BARRIER()   __asm__ volatile("dmb" ::: "memory")

static mqtt_client_t *      g_p_client = NULL;
static ip_addr_t            g_broker_addr;
static volatile uint8_t     g_b_bound = 0u;
static volatile uint8_t     g_b_connecting = 0u;
static int32_t              g_mbf_tx = 0;
static int32_t              g_mbf_rx = 0;
static uint8_t              g_will_msg[MQTT_UTK_WILL_MAX + 1u]; // +1: NUL.
static uint32_t             g_will_len = 0u;
static volatile uint32_t    g_last_contact_ms = 0u;
static volatile uint32_t    g_tx_dropped = 0u;
static volatile uint32_t    g_rx_dropped = 0u;
static uint8_t              g_tx_seq[SEQ_STREAMS];
static uint8_t              g_rx_data[MQTT_UTK_RX_MAX_MSG];
static uint32_t             g_rx_len = 0u;
static bool                 g_b_rx_wanted = false;
static bool                 g_b_rx_overflow = false;

static bool b_client_is_up(void);
static void note_contact(void);
static void flush_tx(void);
static void schedule_retry(void);
static void publish_one(uint8_t * p_msg, int32_t size);
static void tx_poll_timeout(void * p_arg);
static void retry_timeout(void * p_arg);
static void force_reconnect_timeout(void * p_arg);

void
mqtt_utk_bind (int32_t mbf_tx_id, int32_t mbf_rx_id, uint8_t const * p_will,
               uint32_t will_len)
{
    bool b_ok = (NULL != p_will) && (will_len > 0u)
                && (will_len <= MQTT_UTK_WILL_MAX);

    if (0u == g_b_bound)
    {
        g_mbf_tx = mbf_tx_id;
        g_mbf_rx = mbf_rx_id;
        g_will_len = 0u;

        for (uint32_t idx = 0u; b_ok && (idx < will_len); idx++)
        {
            g_will_msg[idx] = p_will[idx];

            if (0u == p_will[idx])
            {
                b_ok = false;       // strlen() would cut the will short.
            }
        }

        if (b_ok)
        {
            g_will_msg[will_len] = 0u;
            g_will_len = will_len;
        }
        else
        {
            tm_printf((UB*)"mqtt: no Last-Will (bad frame)\n");
        }

        BARRIER();                  // Publish everything above first,
        g_b_bound = 1u;             // then allow the radio task to connect.
    }

    return;
}

uint32_t
mqtt_utk_last_contact_ms (void)
{
    return (g_last_contact_ms);
}

uint32_t
mqtt_utk_tx_dropped (void)
{
    return (g_tx_dropped);
}

uint32_t
mqtt_utk_rx_dropped (void)
{
    return (g_rx_dropped);
}

static bool
b_client_is_up (void)
{
    return ((NULL != g_p_client)
            && (0u != mqtt_client_is_connected(g_p_client)));
}

static void
note_contact (void)
{
    uint32_t now = sys_now();   // Same clock as the kernel's tk_get_otm().

    g_last_contact_ms = (0u == now) ? 1u : now;     // 0 means "never".

    return;
}

/* Discard everything queued while there was no connection. Old telemetry is
 * worse than none, and discarding keeps the queue from filling. */
static void
flush_tx (void)
{
    uint8_t  msg[MQTT_UTK_TX_MAX_MSG];
    uint32_t count = 0u;
    bool     b_more = true;

    while (b_more && (count < FLUSH_MAX))
    {
        b_more = (tk_rcv_mbf(g_mbf_tx, msg, TMO_POLL) > 0);
        count++;
    }

    return;
}

/* Hold off reconnecting for RETRY_MS, so a broker that keeps refusing us (or
 * a connect that fails at once) cannot turn into a tight loop. */
static void
schedule_retry (void)
{
    g_b_connecting = 1u;
    sys_untimeout(retry_timeout, NULL);
    sys_timeout(RETRY_MS, retry_timeout, NULL);

    return;
}

static void
retry_timeout (void * p_arg)
{
    (void) p_arg;
    g_b_connecting = 0u;
    mqtt_utk_start();

    return;
}

static void
force_reconnect_timeout (void * p_arg)
{
    (void) p_arg;

    if (b_client_is_up())
    {
        // NOTE: lwIP's mqtt_disconnect() does not call connection_cb(), so
        // stop the TX chain and arm the retry backoff here.
        mqtt_disconnect(g_p_client);
        sys_untimeout(tx_poll_timeout, NULL);
        flush_tx();
        schedule_retry();
    }

    return;
}

/* Called only for QoS 1 publishes: the broker's PUBACK is proof of contact. */
static void
publish_ack_cb (void * p_arg, err_t result)
{
    (void) p_arg;

    if (ERR_OK == result)
    {
        note_contact();
    }

    return;
}

/* msg = flag byte + frame. size is at least 3 (checked by the caller). */
static void
publish_one (uint8_t * p_msg, int32_t size)
{
    uint8_t * p_frame = &p_msg[1];
    // NOTE: size >= 3 here, so size - 1 is positive and the cast is safe.
    uint16_t  frame_len = (uint16_t) (size - 1);
    uint32_t  type = ((uint32_t) p_frame[MQTT_UTK_TYPE_BYTE]
                      >> MQTT_UTK_TYPE_SHIFT) & MQTT_UTK_TYPE_MASK;
    bool      b_hb = (MQTT_UTK_TYPE_HEARTBEAT == type);
    uint32_t  stream = b_hb ? STREAM_HB : STREAM_TLM;
    uint8_t   qos = (0u != (p_msg[0] & MQTT_UTK_TX_FLAG_ACK)) ? 1u : 0u;
    err_t     err;

    // The sequence number is patched here: one counter per topic, one writer.
    // It advances even if the publish fails, so a gap shows up as a loss.
    p_frame[0] = g_tx_seq[stream];
    g_tx_seq[stream]++;

    err = mqtt_publish(g_p_client, b_hb ? TOPIC_HB : TOPIC_TLM, p_frame,
                       frame_len, qos, 0u,
                       (1u == qos) ? publish_ack_cb : NULL, NULL);

    if (ERR_OK != err)
    {
        g_tx_dropped++;
    }

    return;
}

static void
tx_poll_timeout (void * p_arg)
{
    uint8_t  msg[MQTT_UTK_TX_MAX_MSG];
    uint32_t count = 0u;
    bool     b_more = true;

    (void) p_arg;

    if (b_client_is_up())
    {
        while (b_more && (count < TX_BURST))
        {
            int32_t size = tk_rcv_mbf(g_mbf_tx, msg, TMO_POLL);

            if (size > 0)
            {
                if ((size >= 3) && (size <= (int32_t) MQTT_UTK_TX_MAX_MSG))
                {
                    publish_one(msg, size);
                }
                else
                {
                    g_tx_dropped++;     // Too short to hold a frame header.
                }

                count++;
            }
            else
            {
                b_more = false;         // Queue empty.
            }
        }

        sys_timeout(TX_POLL_MS, tx_poll_timeout, NULL);
    }
    else
    {
        flush_tx();     // Chain ends; connection_cb() starts it again.
    }

    return;
}

/* A publish arrives as one topic callback, then one or more data callbacks. */
static void
incoming_publish_cb (void * p_arg, char const * p_topic, u32_t tot_len)
{
    (void) p_arg;

    g_rx_len = 0u;
    g_b_rx_wanted = (0 == strcmp(p_topic, TOPIC_CMD));
    g_b_rx_overflow = (tot_len > MQTT_UTK_RX_MAX_MSG);

    return;
}

static void
incoming_data_cb (void * p_arg, u8_t const * p_data, u16_t len, u8_t flags)
{
    (void) p_arg;

    note_contact();     // Anything from the broker proves it is there.

    for (u16_t idx = 0u; g_b_rx_wanted && (idx < len); idx++)
    {
        if (g_rx_len < MQTT_UTK_RX_MAX_MSG)
        {
            g_rx_data[g_rx_len] = p_data[idx];
            g_rx_len++;
        }
        else
        {
            g_b_rx_overflow = true;
        }
    }

    if (0u != (flags & MQTT_DATA_FLAG_LAST))
    {
        // Empty messages are ignored. An oversize command is forwarded as a
        // 1-byte marker (below); the sequence, version, command and length
        // checks are comms_task's job.
        if (g_b_rx_wanted && (g_rx_len > 0u))
        {
            // NOTE: The marker (length 1) can never pass comms_task's
            // length check.
            uint8_t  oversize_marker = 0u;
            void const * p_msg = g_b_rx_overflow
                                     ? (void const *) &oversize_marker
                                     : (void const *) g_rx_data;
            int32_t  msg_len = g_b_rx_overflow ? 1 : (int32_t) g_rx_len;
            int32_t  err = tk_snd_mbf(g_mbf_rx, p_msg, msg_len, TMO_POLL);

            if (err < 0)
            {
                g_rx_dropped++;
            }
        }

        g_rx_len = 0u;
        g_b_rx_wanted = false;
    }

    return;
}

static void
subscribe_done_cb (void * p_arg, err_t result)
{
    (void) p_arg;

    if (ERR_OK == result)
    {
        tm_printf((UB*)"mqtt: subscribed to " MQTT_TOPIC_BASE "/cmd\n");
    }
    else
    {
        // Deaf to commands (including STOP) is worse than reconnecting.
        // Deferred, because we may be inside lwIP's request handling.
        tm_printf((UB*)"mqtt: subscribe failed, err=%d\n", (int) result);
        sys_untimeout(force_reconnect_timeout, NULL);
        sys_timeout(0u, force_reconnect_timeout, NULL);
    }

    return;
}

static void
connection_cb (mqtt_client_t * p_client, void * p_arg,
               mqtt_connection_status_t status)
{
    (void) p_client;
    (void) p_arg;

    if (MQTT_CONNECT_ACCEPTED == status)
    {
        g_b_connecting = 0u;
        note_contact();                 // CONNACK is contact too.
        flush_tx();                     // Drop frames queued while down.

        tm_printf((UB*)"mqtt: connected to " MQTT_BROKER_IP "\n");

        // lwIP keeps no session: callbacks and subscription are redone on
        // every connect.
        mqtt_set_inpub_callback(g_p_client, incoming_publish_cb,
                                incoming_data_cb, NULL);

        if (ERR_OK != mqtt_subscribe(g_p_client, TOPIC_CMD, CMD_QOS,
                                     subscribe_done_cb, NULL))
        {
            tm_printf((UB*)"mqtt: mqtt_subscribe() refused\n");
            sys_untimeout(force_reconnect_timeout, NULL);
            sys_timeout(0u, force_reconnect_timeout, NULL);
        }

        // Exactly one TX chain: drop any old one before arming a new one.
        sys_untimeout(tx_poll_timeout, NULL);
        sys_timeout(TX_POLL_MS, tx_poll_timeout, NULL);
    }
    else
    {
        tm_printf((UB*)"mqtt: connection status %d, retrying in %us\n",
                  (int) status, RETRY_MS / 1000u);
        sys_untimeout(tx_poll_timeout, NULL);
        flush_tx();
        schedule_retry();
    }

    return;
}

void
mqtt_utk_start (void)
{
    struct mqtt_connect_client_info_t client_info = {0};
    err_t                             err;

    BARRIER();      // Pairs with the barrier in mqtt_utk_bind().

    if ((0u != g_b_bound) && (0u == g_b_connecting)
        && (false == b_client_is_up()))
    {
        if (NULL == g_p_client)
        {
            g_p_client = mqtt_client_new();
        }

        if (NULL == g_p_client)
        {
            tm_printf((UB*)"mqtt: mqtt_client_new() failed\n");
        }
        else if (0 == ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&g_broker_addr)))
        {
            tm_printf((UB*)"mqtt: bad MQTT_BROKER_IP\n");
        }
        else
        {
            IP_SET_TYPE(&g_broker_addr, IPADDR_TYPE_V4);

            client_info.client_id = MQTT_CLIENT_ID;
            client_info.keep_alive = KEEPALIVE_S;

            if (g_will_len > 0u)
            {
                client_info.will_topic = TOPIC_HB;
                // NOTE: The will bytes are non-zero and NUL terminated.
                client_info.will_msg = (char const *) g_will_msg;
                client_info.will_qos = WILL_QOS;
                client_info.will_retain = 0u;
            }

            g_b_connecting = 1u;
            err = mqtt_client_connect(g_p_client, &g_broker_addr,
                                      MQTT_BROKER_PORT, connection_cb, NULL,
                                      &client_info);

            if (ERR_OK != err)
            {
                tm_printf((UB*)"mqtt: mqtt_client_connect() failed, err=%d\n",
                          (int) err);
                schedule_retry();
            }
        }
    }

    return;
}

/*** end of file ***/