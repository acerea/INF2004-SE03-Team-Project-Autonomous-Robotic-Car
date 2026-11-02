/** @file task_comms.c
 *
 * @brief Subsystem 1: comms_task, s1_publish and command validation.
 */
#include <stdint.h>
#include <stdbool.h>
#include "subsystem1.h"
#include <tm/tmonitor.h>
#ifndef S1_DEBUG_CMD
#define S1_DEBUG_CMD (0)   // 1 = print every accepted/rejected command.
#endif

#if TM_MQTT
#include "mqtt_utk.h"
_Static_assert(S1_TLM_MAX_MSG == MQTT_UTK_TX_MAX_MSG, "tx size mismatch");
_Static_assert(S1_TLM_FLAG_ACK == MQTT_UTK_TX_FLAG_ACK, "ack flag mismatch");
_Static_assert(S1_RAW_MAX_MSG == MQTT_UTK_RX_MAX_MSG, "rx size mismatch");
#endif

#if TM_WIFI_CYW43
#include "cyw43_utk.h"
#endif

#define LINK_STATUS_UP      (3)         // NOTE: Must equal CYW43_LINK_UP.
#define MS_PER_SEC          (1000u)
#define SEQ_MASK            (0xFFu)
#define SEQ_WINDOW_MAX      (127u)
#define DROP_TICKS_MAX      (255u)
#define STICKY_FAULTS       (S1_FAULT_CMD_OVERFLOW | S1_FAULT_TLM_DROP)

typedef struct
{
    s1_seq_state_t seq;
    uint32_t       last_hb_ms;
    uint32_t       last_tick_ms;
    uint32_t       last_rx_drops;
    uint32_t       last_tx_drops;
    uint8_t        drop_ticks;
    bool           b_ever_connected;
    uint32_t       down_since_ms;
    bool           b_down;
    uint8_t        cmd_ok;     // Wraps at 256 (heartbeat field width).
    uint8_t        cmd_bad;
} comms_ctx_t;

static uint32_t now_ms(void);
static uint32_t contact_ms(void);
static uint32_t lib_tx_drops(void);
static uint32_t lib_rx_drops(void);
static s1_link_t derive_link(comms_ctx_t * p_ctx);
static uint32_t link_fault_bits(s1_link_t link);
static void update_link_flag(s1_link_t link);
static void send_heartbeat(comms_ctx_t const * p_ctx, s1_link_t link,
                           uint32_t now);
static void forward_command(comms_ctx_t * p_ctx, uint8_t packed);
static void handle_command(comms_ctx_t * p_ctx, uint8_t const * p_raw,
                           uint32_t len);
static s1_link_t escalate_link(comms_ctx_t * p_ctx, s1_link_t link,
                           uint32_t now);
static void tick(comms_ctx_t * p_ctx);

ER
s1_publish (uint8_t const * p_frame, uint32_t n_bytes, bool b_ack)
{
    uint8_t msg[S1_TLM_MAX_MSG] = {0};
    ER      err = E_PAR;

    if ((NULL != p_frame) && (n_bytes > 0u)
        && (n_bytes <= PROTO_MAX_FRAME_BYTES))
    {
        msg[0] = b_ack ? S1_TLM_FLAG_ACK : 0u;

        for (uint32_t idx = 0u; idx < n_bytes; idx++)
        {
            msg[idx + 1u] = p_frame[idx];
        }

        // NOTE: n_bytes + 1 is at most 17, so the cast to INT is safe.
        err = tk_snd_mbf(g_mbf_telemetry, msg, (INT) (n_bytes + 1u), TMO_POL);

        if (E_OK != err)
        {
            // Many writers: tell comms_task through a flag, not a counter.
            (void) tk_set_flg(g_flg_comms, S1_FLG_TLM_DROP);
        }
    }

    return (err);
}

bool
s1_cmd_check (uint8_t const * p_raw, uint32_t len, s1_seq_state_t * p_seq,
              uint8_t * p_packed)
{
    proto_bits_t bits;
    bool         b_accept = false;

    proto_open(&bits, p_raw, len);

    uint32_t seq = proto_get(&bits, PROTO_SEQ_BITS);
    uint32_t ver = proto_get(&bits, PROTO_VERSION_BITS);
    uint32_t cmd = proto_get(&bits, S1_CMD_BITS);
    uint32_t arg = proto_get(&bits, S1_ARG_BITS);

    bool b_valid = (S1_CMD_FRAME_BYTES == len) && (false == bits.b_error)
                   && (PROTO_VERSION == ver) && (cmd <= S1_CMD_MODE);

    uint32_t delta = (seq - (uint32_t) p_seq->last_seq) & SEQ_MASK;
    bool     b_new = (false == p_seq->b_have_seq)
                     || ((delta >= 1u) && (delta <= SEQ_WINDOW_MAX));

    // Idempotent commands are never rejected for their sequence number.
    bool b_always = (S1_CMD_STOP == cmd) || (S1_CMD_ESTOP == cmd)
                    || (S1_CMD_RESET == cmd);

    if (b_valid && b_new)
    {
        p_seq->last_seq   = (uint8_t) seq;  // NOTE: seq is 8 bits wide.
        p_seq->b_have_seq = true;
    }

    if (b_valid && (b_new || b_always))
    {
        *p_packed = s1_cmd_pack(cmd, arg);
        b_accept  = true;
    }

    return (b_accept);
}

void
comms_task (INT stacd, void * exinf)
{
    comms_ctx_t ctx = {{0u, false}, 0u, 0u, 0u, 0u, 0u, false,
                       0u, false, 0u, 0u};
    uint8_t     raw[S1_RAW_MAX_MSG] = {0};
    uint32_t    now = now_ms();
    ER          size = E_OK;

    (void) stacd;
    (void) exinf;

    ctx.last_tick_ms = now;
    ctx.last_hb_ms   = now - S1_HEARTBEAT_MS;   // First heartbeat at once.

    for (;;)
    {
        // Blocks until a raw command arrives, or one tick has passed.
        size = tk_rcv_mbf(g_mbf_cmd_raw, raw, S1_TICK_MS);

        if (size > 0)
        {
            // NOTE: size is positive and at most S1_RAW_MAX_MSG.
            handle_command(&ctx, raw, (uint32_t) size);
        }
        else if (E_TMOUT == size)
        {
            // Normal: no command during this tick.
        }
        else
        {
            CHECK(size, "tk_rcv_mbf");
            (void) tk_dly_tsk(S1_TICK_MS);  // Never spin on an error.
        }

        now = now_ms();

        if ((now - ctx.last_tick_ms) >= S1_TICK_MS)
        {
            ctx.last_tick_ms = now;
            tick(&ctx);
        }
    }
}

static uint32_t
now_ms (void)
{
    SYSTIM otm;

    (void) tk_get_otm(&otm);        // Same clock as lwIP's sys_now().

    return (otm.lo);
}

static uint32_t
contact_ms (void)
{
#if TM_MQTT
    return (mqtt_utk_last_contact_ms());
#else
    return (0u);
#endif
}

static uint32_t
lib_tx_drops (void)
{
#if TM_MQTT
    return (mqtt_utk_tx_dropped());
#else
    return (0u);
#endif
}

static uint32_t
lib_rx_drops (void)
{
#if TM_MQTT
    return (mqtt_utk_rx_dropped());
#else
    return (0u);
#endif
}

static s1_link_t
derive_link (comms_ctx_t * p_ctx)
{
    s1_link_t link = S1_LINK_INIT;

#if TM_WIFI_CYW43
    T_CYW43_UTK_STATUS status;
    uint32_t           contact = contact_ms();  // Read before the clock so
    uint32_t           now     = now_ms();      // that now >= contact.
    bool               b_fresh = false;

    cyw43_utk_get_status(&status);

    if ((0u != contact) && ((now - contact) <= S1_LINK_STALE_MS))
    {
        b_fresh = true;
    }

    if (0u == status.init_complete)
    {
        link = S1_LINK_INIT;
    }
    else if (0 != status.init_result)
    {
        link = S1_LINK_FAULT;
    }
    else if (LINK_STATUS_UP != status.link_status)
    {
        link = p_ctx->b_ever_connected ? S1_LINK_RECONNECTING
                                       : S1_LINK_WIFI_CONNECTING;
    }
    else if (b_fresh)
    {
        link = S1_LINK_CONNECTED;
        p_ctx->b_ever_connected = true;
    }
    else
    {
        link = p_ctx->b_ever_connected ? S1_LINK_LOST
                                       : S1_LINK_MQTT_CONNECTING;
    }
#else
    (void) p_ctx;
#endif

    return (link);
}

static uint32_t
link_fault_bits (s1_link_t link)
{
    uint32_t bits = 0u;

    switch (link)
    {
      case S1_LINK_WIFI_CONNECTING:
      case S1_LINK_RECONNECTING:
      case S1_LINK_FAULT:
        bits = S1_FAULT_WIFI_LOST;
      break;

      case S1_LINK_MQTT_CONNECTING:
      case S1_LINK_LOST:
        bits = S1_FAULT_MQTT_LOST;
      break;

      case S1_LINK_INIT:
      case S1_LINK_CONNECTED:
      case S1_LINK_OFFLINE:
      default:
        bits = 0u;
      break;
    }

    return (bits);
}

static void
update_link_flag (s1_link_t link)
{
    if (S1_LINK_CONNECTED == link)
    {
        (void) tk_clr_flg(g_flg_remote, ~((UINT) S1_FLG_LINK_LOST));
    }
    else
    {
        (void) tk_set_flg(g_flg_remote, S1_FLG_LINK_LOST);
    }

    return;
}

static void
send_heartbeat (comms_ctx_t const * p_ctx, s1_link_t link, uint32_t now)
{
    uint8_t      frame[PROTO_MAX_FRAME_BYTES];
    proto_bits_t bits;

    proto_init(&bits, frame, sizeof(frame));
    proto_begin(&bits, PROTO_TYPE_HEARTBEAT);
    proto_put(&bits, (uint32_t) link, PROTO_HB_LINK_BITS);

    // Uptime wraps at 65536 s, so mask it to the field width on purpose.
    proto_put(&bits, (now / MS_PER_SEC) & ((1u << PROTO_HB_UPTIME_BITS) - 1u),
              PROTO_HB_UPTIME_BITS);
    proto_put(&bits, (uint32_t) p_ctx->drop_ticks, PROTO_HB_DROP_BITS);
    proto_put(&bits, (uint32_t) p_ctx->cmd_ok, PROTO_HB_CMD_OK_BITS);
    proto_put(&bits, (uint32_t) p_ctx->cmd_bad, PROTO_HB_CMD_BAD_BITS);

    // QoS 1: the broker's PUBACK is our proof of broker contact.
    (void) s1_publish(frame, proto_end(&bits), true);

    return;
}

static void
forward_command (comms_ctx_t * p_ctx, uint8_t packed)
{
    uint32_t cmd = s1_cmd_of(packed);
    ER       err = E_OK;

    // STOP and ESTOP also set a flag, so a full buffer can never lose them.
    if (S1_CMD_ESTOP == cmd)
    {
        (void) tk_set_flg(g_flg_remote, S1_FLG_ESTOP);
    }
    else if (S1_CMD_STOP == cmd)
    {
        (void) tk_set_flg(g_flg_remote, S1_FLG_STOP);
    }
    else if (S1_CMD_RESET == cmd)
    {
        // NOTE: Only the low 4 bits are used, so the cast cannot truncate.
        g_s1_fault_bits = (uint8_t) (((uint32_t) g_s1_fault_bits)
                                     & ~((uint32_t) STICKY_FAULTS));
        p_ctx->drop_ticks     = 0u;
        p_ctx->seq.b_have_seq = false;      // A restarted host starts afresh.
    }
    else
    {
        // START and MODE need nothing extra here.
    }

    // NOTE: sizeof(packed) is 1, so the cast to INT is safe.
    err = tk_snd_mbf(g_mbf_remote_cmd, &packed, (INT) sizeof(packed), TMO_POL);

    if (E_OK == err)
    {
        (void) tk_set_flg(g_flg_remote, S1_FLG_CMD_READY);
    }
    else
    {
        // NOTE: Low 4 bits only; the cast cannot truncate.
        g_s1_fault_bits = (uint8_t) (((uint32_t) g_s1_fault_bits)
                                     | S1_FAULT_CMD_OVERFLOW);
    }

    return;
}

static void
handle_command (comms_ctx_t * p_ctx, uint8_t const * p_raw, uint32_t len)
{
    uint8_t packed = 0u;

    if (s1_cmd_check(p_raw, len, &p_ctx->seq, &packed))
    {
        p_ctx->cmd_ok = (uint8_t) (p_ctx->cmd_ok + 1u);     // Wraps
#if S1_DEBUG_CMD
        tm_printf((UB*)"s1: cmd accepted cmd=%d arg=%d\n",
                  (int) s1_cmd_of(packed), (int) s1_arg_of(packed));
#endif
        forward_command(p_ctx, packed);
    }
    else
    {
        p_ctx->cmd_bad = (uint8_t) (p_ctx->cmd_bad + 1u);   // Wraps
#if S1_DEBUG_CMD
        tm_printf((UB*)"s1: cmd rejected\n");
#endif
    }

    return;
}

static s1_link_t
escalate_link (comms_ctx_t * p_ctx, s1_link_t link, uint32_t now)
{
    if ((S1_LINK_LOST == link) || (S1_LINK_RECONNECTING == link))
    {
        if (false == p_ctx->b_down)
        {
            p_ctx->b_down        = true;
            p_ctx->down_since_ms = now;
        }
        else if ((now - p_ctx->down_since_ms) >= S1_LINK_FAULT_MS)
        {
            link = S1_LINK_FAULT;   // Lib keeps retrying.
        }
        else
        {
            // Still inside the 10 s reconnect window.
        }
    }
    else
    {
        p_ctx->b_down = false;
    }

    return (link);
}

static void
tick (comms_ctx_t * p_ctx)
{
    s1_link_t link     = derive_link(p_ctx);
    uint32_t  now      = now_ms();
    uint32_t  fault    = ((uint32_t) g_s1_fault_bits) & STICKY_FAULTS;
    uint32_t  tx_drops = lib_tx_drops();
    uint32_t  rx_drops = lib_rx_drops();
    bool      b_drop   = false;
    UINT      ptn      = 0u;
    ER        err      = E_OK;

    update_link_flag(link);

    // Frames dropped by any publisher (many writers) arrive as a flag.
    err = tk_wai_flg(g_flg_comms, S1_FLG_TLM_DROP, TWF_ORW | TWF_CLR, &ptn,
                     TMO_POL);
    if (E_OK == err)
    {
        b_drop = true;
    }

    // Library counters have one writer each; detect a change since last tick.
    if (tx_drops != p_ctx->last_tx_drops)
    {
        p_ctx->last_tx_drops = tx_drops;
        b_drop = true;
    }

    if (rx_drops != p_ctx->last_rx_drops)
    {
        p_ctx->last_rx_drops = rx_drops;
        fault |= S1_FAULT_CMD_OVERFLOW;
    }

    // While the link is down, drops are expected (nothing drains the queue)
    // and are already reported as WIFI_LOST / MQTT_LOST, so only count real
    // drops while connected.
    if (b_drop && (S1_LINK_CONNECTED == link))
    {
        fault |= S1_FAULT_TLM_DROP;

        if (p_ctx->drop_ticks < DROP_TICKS_MAX)
        {
            p_ctx->drop_ticks++;
        }
    }

    fault |= link_fault_bits(link);
    link   = escalate_link(p_ctx, link, now);

    // NOTE: Fault bits occupy the low 4 bits only; the cast cannot truncate.
    g_s1_fault_bits = (uint8_t) fault;

    if ((now - p_ctx->last_hb_ms) >= S1_HEARTBEAT_MS)
    {
        p_ctx->last_hb_ms = now;
        send_heartbeat(p_ctx, link, now);
    }

    return;
}

/*** end of file ***/