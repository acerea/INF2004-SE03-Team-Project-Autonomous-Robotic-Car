/** @file ir_sense.c
 *
 * @brief Subsystem 3: IR line sensing and barcode reading task.
 *
 * @par
 * ir_sensor_task is the only task that uses the ADC. It runs a state
 * machine (one handler per state, table-driven):
 *
 *   CALI       0.5 s at power-up: learn the white and black levels from
 *              the min/max of all three sensors (place the robot ON the
 *              line, so the centre sees black and the sides see white).
 *              Too little contrast -> defaults, with a warning.
 *   TRACK      Every 10 ms: classify each sensor (hysteresis), estimate
 *              the line position, detect junctions. All three black ->
 *              BARCODE (bars span the track). None black -> LINE_LOST.
 *   BARCODE    Every 2 ms: time the black/white elements seen by the two
 *              SIDE sensors (a bar = both black). The side sensors are
 *              used because the guide line may run through the barcode,
 *              in which case the centre sensor never sees the white
 *              spaces. Every 9 elements -> one Code 39 character, the 3
 *              widest are "wide" (speed-independent). White for longer
 *              than 4 narrow elements (the quiet zone) ends the barcode: decode the word (either direction), map
 *              it to a navigation command, report it, back to TRACK.
 *              Black for > IR_BLACK_MAX_MS is not a barcode: junction
 *              (T / cross / solid pad); no new barcode starts until the
 *              side sensors have seen white again.
 *   Junctions are held for IR_JUNC_HOLD_MS so a 10 ms poll cannot miss
 *   them.
 *   LINE_LOST  All three black -> BARCODE (white gap before a barcode).
 *              Any sensor black again -> TRACK. Lost for longer than
 *              IR_LOST_TIMEOUT_MS -> lost_timeout bit in the snapshot.
 *
 * Shared data: the snapshot (ir_snapshot_t) is written once per tick
 * under ir_mutex_id and read by ir_get_telemetry() under the same mutex.
 * The decoder context is private to the task, so it needs no lock.
 */
#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "subsystem3.h"
#include "ir_sense.h"
#include "ir_algo.h"
#include "../subsystem1/subsystem1.h"

// ADC channels, as the car is actually wired (TEAM.md section 2):
//   GP26 = ADC0 = RIGHT line sensor
//   GP27 = ADC1 = middle (centre) line sensor
//   GP28 = ADC2 = LEFT sensor (the barcode sensor)
// All three must be on the boards' analog AO outputs, not DO: the code
// measures darkness, and DO only gives on/off. The old mapping (ADC0 =
// left) reversed the steering on this car.
#define ADC_LEFT                (2)
#define ADC_CENTRE              (1)
#define ADC_RIGHT               (0)

#define IR_TRACK_MS             (10u)       // Line tracking period.
#define IR_BARCODE_MS           (2u)        // Barcode sampling period.
#define IR_CAL_SAMPLES          (50u)       // 0.5 s calibration.
#define IR_LOST_TIMEOUT_MS      (1000u)     // Line lost -> report.
#define IR_BLACK_MAX_MS         (1000u)     // Longer black: not a barcode.
#define IR_BARCODE_MAX_MS       (12000u)    // Give up on a barcode (141 mm
                                            // at ~1.2 cm/s).
#define IR_QUIET_MIN_MS         (60u)       // Shortest end-of-barcode gap.
#define IR_QUIET_MAX_MS         (2500u)
#define IR_PEAK_EDGES           (1)         // 1 = edges relative to each
                                            // element's own peak (below).
#define IR_PEAK_KEEP_PCT        (70u)       // Bar ends below 70 % of its peak.
#define IR_PEAK_RISE_PCT        (30u)       // Space ends 30 % (of the last
                                            // bar's peak) above its lowest.
#define IR_PEAK_MIN_SWING       (80u)       // ADC counts; smaller = noise.
#define IR_DEBOUNCE_MS          (20u)       // A colour change must last this
                                            // long to count (edge flicker
                                            // made 16 ms fake elements).
                                            // Fine below ~12 cm/s.     // Longest end-of-barcode wait.
#define IR_QUIET_WIDEST         (4u)        // Quiet zone > 4 x the widest
                                            // element so far. Based on the
                                            // widest, not the narrowest: one
                                            // short (fast) element used to
                                            // make the limit so small that
                                            // the next wide space ended the
                                            // barcode.
#define IR_JUNC_SAMPLES         (3u)        // Junction must hold 30 ms.
#define IR_BARCODE_CONFIRM_SAMPLES (3u)     // Barcode pad must hold 30 ms.
#define IR_JUNC_HOLD_MS         (300u)      // Report a junction this long.
#define IR_LINE_PUB_MS          (1000u)     // LINE frame at least every 1 s.
#define IR_POS_PUB_MS           (100u)      // Line position alone: max 10 Hz.
#define IR_MAX_CHARS            (10u)       // '*' + 8 letters + '*'.
#define IR_WORD_MAX             (IR_MAX_CHARS + 1u)

// Decoder and tracker context, private to ir_sensor_task. Counters use
// the bits their range needs (change requirement 1).
typedef struct
{
    ir_cal_t cal[3];                    // White / black per sensor: L, C, R.
    ir_cal_t cal_saved[3];              // Levels before a sweep (cancel).
    uint16_t sw_lo[3];                  // Sweep min / max per sensor.
    uint16_t sw_hi[3];
    uint16_t cal_lo;                    // Static calibration min / max
    uint16_t cal_hi;                    // (all sensors together).
    bool     b_sweeping;                // Sweep calibration running.
    uint32_t state_ms;                  // Time the state was entered.
    uint32_t edge_ms;                   // Last barcode transition.
    uint32_t last_pub_ms;               // Last LINE frame.
    uint32_t junc_ms;                   // Time the junction was seen.
    uint32_t bar_first_ms;              // First all-black sample of a pad.
    uint16_t widths[IR_C39_ELEMENTS];   // Element widths, ms.
    uint16_t pats[IR_MAX_CHARS];        // 9-bit character patterns.
    uint16_t min_elem_ms;               // Narrowest element so far.
    uint16_t max_elem_ms;               // Widest element so far.
    int16_t  last_line_pos;             // Last real line position for reacquisition.
    uint8_t  cal_count;                 // 0..50.
    uint8_t  elem    : 4;               // 0..9 elements of the character.
    uint8_t  chars   : 4;               // 0..10 characters read.
    uint8_t  b_black : 1;               // Centre sensor black (barcode).
    uint8_t  b_gap   : 1;               // In an inter-character gap.
    uint8_t  junc_run: 2;               // 0..3 junction samples.
    uint8_t  barcode_run: 2;            // All-black confirmation samples.
    uint8_t  sensors : 3;               // Last IR_SENS_xxx bits.
    uint8_t  junc    : 2;               // Junction being reported.
    uint8_t  b_armed : 1;               // Sides saw white since last pad.
} ir_ctx_t;

typedef void (*ir_handler_t)(uint32_t now);

static ir_ctx_t       g_ctx;

// Scan record for the SCAN telemetry (type 10): every element of the
// current read attempt, in ms, in the order seen. Task-private.
static uint16_t       g_scan_ms[PROTO_SC_MAX_ELEMS];
static uint8_t        g_scan_n = 0u;
static uint16_t       g_scan_narrow_min = 0xFFFFu;  // Of decoded chars.
static uint16_t       g_scan_narrow_max = 0u;
static bool           g_b_scan_timeout = false;
static bool           g_b_edge_pending = false; // Colour change being debounced.
// Peak-relative edge tracker (IR_PEAK_EDGES), in ADC counts above white.
static bool           g_b_trk_black = false;
static uint16_t       g_trk_peak = 0u;      // Darkest of the current bar.
static uint16_t       g_trk_trough = 0u;    // Lightest of the current space.
static uint16_t       g_trk_ref = 0u;       // Peak of the last bar.
static uint32_t       g_edge_pending_ms = 0u;   // Its first sample.
static ir_snapshot_t  g_snap;           // Shared, under g_ir_mutex.
static ir_snapshot_t  g_next;           // Built by the task each tick.
static ID             g_ir_mutex = -1;
static ID             g_adc_dd = -1;
static bool           g_b_ready = false;
static uint8_t        g_last_pub_state = 0xFFu;
static uint8_t        g_last_pub_junc = 0xFFu;
static uint8_t        g_last_pub_sensors = 0xFFu;
static int16_t        g_last_pub_pos = -32768;
static volatile bool  g_b_steer_locked = false;
static volatile bool  g_b_lift_override = false;
static volatile bool  g_b_recal_request = false;

// DEBUG: breadcrumb of the IR task's position, read by the VC when the IR
// task stops updating. stage | (ADC channel << 8).
//   1 loop top        2 handler          3 before tk_srea_dev (ADC read)
//   4 after ADC read  5 before mutex     6 after mutex
//   7 publish line    8 in tk_dly_tsk    9 sweep requests
static volatile uint32_t g_ir_stage = 0u;
#define IR_STAGE(s)   (g_ir_stage = (s))
static volatile bool  g_b_sweep_begin = false;
// Barcode sensor's last black/white decision, for hysteresis. Only
// ir_sensor_task reads or writes it.
static bool           g_b_bar_black = false;
// DEBUG: last raw ADC readings (L, C, R) for the periodic RAW print.
static uint16_t       g_dbg_raw[3] = {0u, 0u, 0u};
static uint32_t       g_dbg_ms = 0u;
#define IR_DBG_RAW_MS           (500u)      // RAW print period.
static volatile bool  g_b_sweep_end = false;
static volatile bool  g_b_sweep_cancel = false;

EXPORT ID mbf_s3_nav;                   // IF-15.

static void ir_handle_cali(uint32_t now);
static void s3_publish_ircal(uint32_t kind, uint32_t ok_mask);
static void ir_handle_track(uint32_t now);
static void ir_handle_barcode(uint32_t now);
static void ir_handle_lost(uint32_t now);
static uint16_t ir_bar_lift(void);
static bool ir_peak_update(uint16_t lift);
static void ir_scan_tx_poll(void);

static ir_handler_t const g_handlers[] =
{
    [IR_CALI]      = ir_handle_cali,
    [IR_TRACK]     = ir_handle_track,
    [IR_BARCODE]   = ir_handle_barcode,
    [IR_LINE_LOST] = ir_handle_lost,
};

static uint32_t
ir_now_ms (void)
{
    SYSTIM now;

    (void) tk_get_otm(&now);
    return (now.lo);
}

static uint16_t
ir_read_adc (INT channel)
{
    UW  value = 0u;
    SZ  asize = 0;
    ER  err;

    static uint8_t s_err_prints = 0u;       // DEBUG: first errors only.
    static uint8_t s_ok_seen = 0u;          // DEBUG: bit per channel.

    IR_STAGE(3u | ((uint32_t) channel << 8));
    err = tk_srea_dev(g_adc_dd, channel, &value, 1, &asize);
    IR_STAGE(4u | ((uint32_t) channel << 8));
    if ((err < E_OK) || (asize < 1))
    {
        if (s_err_prints < 6u)
        {
            s_err_prints++;
            tm_printf((UB*)"ir: DEBUG ADC ch%d read failed err=%d size=%d CS=0x%08x\n",
                      (int) channel, (int) err, (int) asize,
                      (unsigned int) (*(volatile UW *) 0x4004C000u));
        }
        value = 0u;     // Reads as white: a dead sensor looks like no line.
    }
    else if (0u == (s_ok_seen & (1u << channel)))
    {
        s_ok_seen |= (uint8_t) (1u << channel);
        tm_printf((UB*)"ir: DEBUG ADC ch%d first reading %u\n",
                  (int) channel, (unsigned int) (value & IR_ADC_MAX));
    }
    else
    {
        // Channel already reported once: nothing to print.
    }

    return ((uint16_t) (value & IR_ADC_MAX));
}

static void
ir_set_state (uint8_t state, uint32_t now)
{
    g_next.state   = state & IR_MASK;
    g_ctx.state_ms = now;

    return;
}

// Line-following uses the centre and right IR sensors only.  ADC_LEFT is
// the barcode sensor and must not contribute to line_pos or steering.
static uint8_t
ir_read_line_sensors (uint16_t p_dark[2])
{
    uint16_t raw_c = ir_read_adc(ADC_CENTRE);
    uint16_t raw_r = ir_read_adc(ADC_RIGHT);
    uint8_t old = g_ctx.sensors;
    uint8_t bits = 0u;

    g_dbg_raw[1] = raw_c;
    g_dbg_raw[2] = raw_r;

    if (ir_is_black(raw_c, 0u != (old & IR_SENS_CENTRE), &g_ctx.cal[1]))
    {
        bits |= IR_SENS_CENTRE;
    }
    if (ir_is_black(raw_r, 0u != (old & IR_SENS_RIGHT), &g_ctx.cal[2]))
    {
        bits |= IR_SENS_RIGHT;
    }

    if (NULL != p_dark)
    {
        p_dark[0] = ir_darkness(raw_c, &g_ctx.cal[1]);
        p_dark[1] = ir_darkness(raw_r, &g_ctx.cal[2]);
    }

    g_ctx.sensors = (g_ctx.sensors & ~((uint8_t)(IR_SENS_CENTRE | IR_SENS_RIGHT)))
                    | bits;
    return (bits);
}

// Barcode sampling reads only the dedicated left/barcode sensor.
// It does not update the line snapshot or line correction.
static uint8_t
ir_read_barcode_sensor (void)
{
    uint16_t raw_l = ir_read_adc(ADC_LEFT);
    uint8_t bits = 0u;

    g_dbg_raw[0] = raw_l;
    // Hysteresis (black >= 45 %, white <= 30 %, keep in between): without
    // it, ADC noise at a bar edge flips the result several times and adds
    // false 2-20 ms elements that shift the 9-element character framing.
    g_b_bar_black = ir_is_bar_black(raw_l, g_b_bar_black, &g_ctx.cal[0]);
    if (g_b_bar_black)
    {
        bits |= IR_SENS_LEFT;
    }

    return (bits);
}

static void
ir_handle_cali (uint32_t now)
{
    uint16_t raw[3];
    uint8_t  idx;

    raw[0] = ir_read_adc(ADC_LEFT);
    raw[1] = ir_read_adc(ADC_CENTRE);
    raw[2] = ir_read_adc(ADC_RIGHT);

    // Sweep: each sensor keeps its own range; the VC ends the sweep.
    if (g_ctx.b_sweeping)
    {
        for (idx = 0u; idx < 3u; idx++)
        {
            if (raw[idx] < g_ctx.sw_lo[idx])
            {
                g_ctx.sw_lo[idx] = raw[idx];
            }
            if (raw[idx] > g_ctx.sw_hi[idx])
            {
                g_ctx.sw_hi[idx] = raw[idx];
            }
        }
        return;
    }

    if (0u == g_ctx.cal_count)
    {
        g_ctx.cal_lo = IR_ADC_MAX;
        g_ctx.cal_hi = 0u;
    }

    for (idx = 0u; idx < 3u; idx++)
    {
        if (raw[idx] < g_ctx.cal_lo)
        {
            g_ctx.cal_lo = raw[idx];
        }
        if (raw[idx] > g_ctx.cal_hi)
        {
            g_ctx.cal_hi = raw[idx];
        }
    }

    g_ctx.cal_count++;
    if (g_ctx.cal_count >= IR_CAL_SAMPLES)
    {
        // Standing still on the line, each sensor sees only one colour,
        // so the static calibration uses one range for all three.
        ir_cal_set(&g_ctx.cal[0], g_ctx.cal_lo, g_ctx.cal_hi);
        g_ctx.cal[1] = g_ctx.cal[0];
        g_ctx.cal[2] = g_ctx.cal[0];

        if ((g_ctx.cal[0].white != g_ctx.cal_lo)
            || (g_ctx.cal[0].black != g_ctx.cal_hi))
        {
            tm_printf((UB*)"ir: low contrast (%u..%u), default levels\n",
                      (unsigned int) g_ctx.cal_lo,
                      (unsigned int) g_ctx.cal_hi);
        }
        tm_printf((UB*)"ir: calibrated white=%u black=%u\n",
                  (unsigned int) g_ctx.cal[0].white,
                  (unsigned int) g_ctx.cal[0].black);
        s3_publish_ircal(0u, ((g_ctx.cal[0].white == g_ctx.cal_lo)
                              && (g_ctx.cal[0].black == g_ctx.cal_hi))
                             ? 7u : 0u);
        ir_set_state(IR_TRACK, now);
    }

    return;
}

// start_ms is the first all-black sample, not the confirming one: the
// confirmation (IR_BARCODE_CONFIRM_SAMPLES ticks) used to be cut off the
// first bar, which made min_elem_ms (and the quiet-zone limit) too small
// and could end the barcode at its first wide space.
static void
ir_barcode_begin (uint32_t now, uint32_t start_ms)
{
    g_ctx.edge_ms     = start_ms;
    g_ctx.elem        = 0u;
    g_ctx.chars       = 0u;
    g_ctx.b_black     = 1u;
    g_ctx.b_gap       = 0u;
    g_ctx.min_elem_ms = 0xFFFFu;
    g_ctx.max_elem_ms = 0u;
    g_scan_n          = 0u;
    g_scan_narrow_min = 0xFFFFu;
    g_scan_narrow_max = 0u;
    g_b_scan_timeout  = false;
    g_b_edge_pending  = false;
    // The tracker starts on the bar that started the barcode.
    g_b_trk_black     = true;
    g_trk_peak        = ir_bar_lift();
    g_trk_ref         = g_trk_peak;
    g_trk_trough      = 0u;
    ir_set_state(IR_BARCODE, now);

    return;
}

static void
ir_handle_track (uint32_t now)
{
    if (g_b_lift_override)
    {
        ir_set_line_pos(&g_next, 0);
        g_next.sensors = 0u;
        g_next.junction = IR_JUNC_NONE;
        g_next.lost_timeout = 1u;
        ir_set_state(IR_LINE_LOST, now);
        return;
    }

    uint16_t dark[2];
    uint8_t  line_bits = ir_read_line_sensors(dark);
    uint8_t  barcode = ir_read_barcode_sensor();
    uint8_t  junc = IR_JUNC_NONE;
    int16_t  pos = 0;

    g_next.sensors = line_bits;

    if (0u == (barcode & IR_SENS_LEFT))
    {
        g_ctx.b_armed = 1u;             // Off any bar or pad.
    }

    // All three black must persist briefly before treating it as a barcode.
    // This distinguishes a barcode start from a short line/junction crossing.
    if ((0u != (barcode & IR_SENS_LEFT))
        && (0u != g_ctx.b_armed)
        && (!g_b_steer_locked))
    {
        if (0u == g_ctx.barcode_run)
        {
            g_ctx.bar_first_ms = now;
        }
        if (g_ctx.barcode_run < IR_BARCODE_CONFIRM_SAMPLES)
        {
            g_ctx.barcode_run++;
        }
        if (g_ctx.barcode_run >= IR_BARCODE_CONFIRM_SAMPLES)
        {
            ir_barcode_begin(now, g_ctx.bar_first_ms);
            g_ctx.barcode_run = 0u;
            return;
        }
    }
    else
    {
        g_ctx.barcode_run = 0u;
    }

    if (0u == line_bits)
    {
        g_next.lost_timeout = 0u;
        ir_set_line_pos(&g_next, g_ctx.last_line_pos);
        ir_set_state(IR_LINE_LOST, now);
        return;
    }

    g_next.lost_timeout = 0u;
    ir_set_line_pos(&g_next, 0);
    if (ir_line_position_lr(dark[0], dark[1], &pos))
    {
        ir_set_line_pos(&g_next, pos);      // |pos| <= 1000: fits 11 bits.
        g_ctx.last_line_pos = pos;
    }

    // Junction/branch reporting uses only the line-sensor pair.
    if (line_bits == (IR_SENS_RIGHT | IR_SENS_CENTRE))
    {
        junc = IR_JUNC_RIGHT;
    }
    else
    {
        junc = IR_JUNC_NONE;
    }

    if ((IR_JUNC_NONE != junc) && (g_ctx.junc_run < IR_JUNC_SAMPLES))
    {
        g_ctx.junc_run++;
    }
    else if (IR_JUNC_NONE == junc)
    {
        g_ctx.junc_run = 0u;
    }
    else
    {
        // Already counted.
    }

    if (g_ctx.junc_run >= IR_JUNC_SAMPLES)
    {
        g_ctx.junc    = junc & 0x03u;
        g_ctx.junc_ms = now;
    }
    else if ((now - g_ctx.junc_ms) > IR_JUNC_HOLD_MS)
    {
        g_ctx.junc = IR_JUNC_NONE;
    }
    else
    {
        // Keep reporting the last junction for a while.
    }

    g_next.junction = g_ctx.junc;
    return;
}

// BARCODE frame (type 4), as specified by subsystem 1's s3_publish_barcode
// helper: letter 5 bits (0-25, 31 = unreadable), command 3 bits, QoS 0.
static void
s3_publish_barcode (char letter, uint32_t command)
{
    uint8_t      frame[PROTO_MAX_FRAME_BYTES];
    proto_bits_t bits;
    uint32_t     code = PROTO_BC_LETTER_BAD;    // Not A-Z: unreadable.

    if ((letter >= 'A') && (letter <= 'Z'))
    {
        code = (uint32_t) (letter - 'A');       // 'A' = 0 ... 'Z' = 25.
    }

    proto_init(&bits, frame, sizeof(frame));
    proto_begin(&bits, PROTO_TYPE_BARCODE);
    proto_put(&bits, code, PROTO_BC_LETTER_BITS);
    proto_put(&bits, command, PROTO_BC_CMD_BITS);
    (void) s1_publish(frame, proto_end(&bits), false);

    return;
}

// SCAN frames (type 10): the elements of one read attempt and the result.
// Widths are sent in units of "scale" ms so each fits 6 bits.
//
// Sent one frame per TRACK tick from a copy (ir_scan_tx_poll), and a
// frame the telemetry queue refused is retried: sent all at once, the 2-4
// frames often found the queue full and the scan never reached the
// dashboard (test 25 Nov: 6 frames lost after one '?' read).
#define IR_SCAN_TX_TRIES    (100u)      // ~1 s of retries per frame.

typedef struct
{
    uint16_t ms[PROTO_SC_MAX_ELEMS];
    uint8_t  n;
    uint8_t  part;          // Next part to send.
    uint8_t  tries;
    uint8_t  reason : 3;
    uint8_t  uneven : 1;
    uint8_t  b_busy : 1;    // Frames still to send.
    uint8_t  scale;
} ir_scan_tx_t;

static ir_scan_tx_t g_scan_tx;

static void
s3_publish_scan (uint32_t reason)
{
    uint32_t widest = 1u;
    uint32_t scale;
    uint8_t  idx;

    // A lone bar or pad (noise, a junction bar, standing on black) is
    // only printed: sending it filled the queue for nothing.
    if (g_scan_n <= 1u)
    {
        return;
    }

    for (idx = 0u; idx < g_scan_n; idx++)
    {
        g_scan_tx.ms[idx] = g_scan_ms[idx];
        if (g_scan_ms[idx] > widest)
        {
            widest = g_scan_ms[idx];
        }
    }
    scale = (widest + 62u) / 63u;           // Round up: widest fits 63.
    if (scale > 255u)
    {
        scale = 255u;
    }

    g_scan_tx.n      = g_scan_n;
    g_scan_tx.scale  = (uint8_t) scale;
    g_scan_tx.reason = reason & 0x07u;
    g_scan_tx.uneven = ((0xFFFFu != g_scan_narrow_min)
                        && ((uint32_t) g_scan_narrow_max
                            > (2u * (uint32_t) g_scan_narrow_min))) ? 1u : 0u;
    g_scan_tx.part   = 0u;
    g_scan_tx.tries  = 0u;
    g_scan_tx.b_busy = 1u;                  // Replaces an unsent older one.

    return;
}

// Send the next SCAN frame, if any. Never while a barcode is being timed.
static void
ir_scan_tx_poll (void)
{
    uint8_t      frame[PROTO_MAX_FRAME_BYTES];
    proto_bits_t bits;
    uint8_t      first;
    uint8_t      count;
    uint8_t      idx;
    bool         b_last;

    if (0u == g_scan_tx.b_busy)
    {
        return;
    }

    first = (uint8_t) (g_scan_tx.part * PROTO_SC_PER_PART);
    count = (g_scan_tx.n > first) ? (uint8_t) (g_scan_tx.n - first) : 0u;
    if (count > PROTO_SC_PER_PART)
    {
        count = PROTO_SC_PER_PART;
    }
    b_last = ((first + count) >= g_scan_tx.n)
             || ((g_scan_tx.part + 1u)
                 >= (PROTO_SC_MAX_ELEMS / PROTO_SC_PER_PART));

    proto_init(&bits, frame, sizeof(frame));
    proto_begin(&bits, PROTO_TYPE_SCAN);
    proto_put(&bits, g_scan_tx.part, PROTO_SC_PART_BITS);
    proto_put(&bits, b_last ? 1u : 0u, PROTO_SC_LAST_BITS);
    proto_put(&bits, g_scan_tx.reason, PROTO_SC_REASON_BITS);
    proto_put(&bits, g_scan_tx.uneven, PROTO_SC_UNEVEN_BITS);
    proto_put(&bits, g_scan_tx.scale, PROTO_SC_SCALE_BITS);
    proto_put(&bits, count, PROTO_SC_COUNT_BITS);
    for (idx = 0u; idx < count; idx++)
    {
        uint32_t ms = g_scan_tx.ms[first + idx];
        uint32_t w  = (ms + (g_scan_tx.scale / 2u)) / g_scan_tx.scale;

        if ((0u == w) && (0u != ms))
        {
            w = 1u;
        }
        proto_put_sat(&bits, w, PROTO_SC_WIDTH_BITS);
    }

    if (E_OK == s1_publish(frame, proto_end(&bits), false))
    {
        g_scan_tx.tries = 0u;
        g_scan_tx.part++;
        if (b_last)
        {
            g_scan_tx.b_busy = 0u;
        }
    }
    else if (++g_scan_tx.tries >= IR_SCAN_TX_TRIES)
    {
        g_scan_tx.b_busy = 0u;              // Give up on this scan.
    }
    else
    {
        // Queue full: try the same frame on the next tick.
    }

    return;
}

// Barcode sensor reading in ADC counts above its calibrated white.
static uint16_t
ir_bar_lift (void)
{
    uint16_t raw = g_dbg_raw[0];            // Set by ir_read_barcode_sensor.

    return ((raw > g_ctx.cal[0].white) ? (uint16_t) (raw - g_ctx.cal[0].white)
                                        : 0u);
}

// Edges relative to each element's own extreme instead of a fixed level.
// The sensor spot is wider than a narrow bar, so a narrow bar never gets
// as dark as a wide one: with a fixed level, narrow bars read thin and
// were sometimes missed (test 25 Nov). A bar ends when the reading falls
// below IR_PEAK_KEEP_PCT of that bar's own peak; a space ends when it
// rises IR_PEAK_RISE_PCT of the last bar's peak above the space's lowest
// point. Both need IR_PEAK_MIN_SWING counts, so ADC noise cannot flip it.
static bool
ir_peak_update (uint16_t lift)
{
    if (g_b_trk_black)
    {
        if (lift > g_trk_peak)
        {
            g_trk_peak = lift;
        }
        if ((((uint32_t) lift * 100u)
             < ((uint32_t) g_trk_peak * IR_PEAK_KEEP_PCT))
            && ((uint32_t) (g_trk_peak - lift) >= IR_PEAK_MIN_SWING))
        {
            g_b_trk_black = false;
            g_trk_ref     = g_trk_peak;
            g_trk_trough  = lift;
        }
    }
    else
    {
        if (lift < g_trk_trough)
        {
            g_trk_trough = lift;
        }
        if ((((uint32_t) lift)
             > ((uint32_t) g_trk_trough
                + (((uint32_t) g_trk_ref * IR_PEAK_RISE_PCT) / 100u)))
            && ((uint32_t) (lift - g_trk_trough) >= IR_PEAK_MIN_SWING))
        {
            g_b_trk_black = true;
            g_trk_peak    = lift;
        }
    }

    return (g_b_trk_black);
}

// Why a read attempt that did not decode failed (PROTO_SCAN_xxx).
static uint32_t
ir_scan_reason (uint8_t len)
{
    uint8_t idx;

    if (len > 0u)
    {
        return (PROTO_SCAN_OK);
    }
    if (g_b_scan_timeout)
    {
        return (PROTO_SCAN_TIMEOUT);
    }
    if (0u == g_ctx.chars)
    {
        return (PROTO_SCAN_NO_CHAR);
    }
    if (0u != g_ctx.elem)
    {
        return (PROTO_SCAN_EXTRA);          // Framing slipped somewhere.
    }
    for (idx = 0u; idx < g_ctx.chars; idx++)
    {
        if (0u == g_ctx.pats[idx])
        {
            return (PROTO_SCAN_UNCLEAR);
        }
    }
    for (idx = 0u; idx < g_ctx.chars; idx++)
    {
        if (IR_C39_BAD == ir_c39_lookup(g_ctx.pats[idx]))
        {
            return (PROTO_SCAN_BAD_CHAR);
        }
    }
    return (PROTO_SCAN_NO_FRAME);
}

static char const * const g_scan_reason_text[8] =
{
    "OK", "single bar/noise", "missed or extra bar", "wide/narrow unclear",
    "not a Code 39 char", "no * start/stop", "solid black (junction)",
    "timed out"
};

// IRCAL frame (type 8): the calibration result, so it can be checked on
// the dashboard / station log without the serial port. ok_mask bit 0 = L,
// bit 1 = C, bit 2 = R: 1 if that sensor got new levels.
static void
s3_publish_ircal (uint32_t kind, uint32_t ok_mask)
{
    uint8_t      frame[PROTO_MAX_FRAME_BYTES];
    proto_bits_t bits;
    uint8_t      idx;

    proto_init(&bits, frame, sizeof(frame));
    proto_begin(&bits, PROTO_TYPE_IRCAL);
    proto_put(&bits, kind, PROTO_IRC_KIND_BITS);
    for (idx = 0u; idx < 3u; idx++)
    {
        proto_put_sat(&bits, g_ctx.cal[idx].white, PROTO_IRC_LEVEL_BITS);
        proto_put_sat(&bits, g_ctx.cal[idx].black, PROTO_IRC_LEVEL_BITS);
        proto_put(&bits, (ok_mask >> idx) & 1u, PROTO_IRC_OK_BITS);
    }
    (void) s1_publish(frame, proto_end(&bits), false);

    return;
}

static void
ir_barcode_finish (uint32_t now)
{
    char         word[IR_WORD_MAX];
    uint8_t      len;
    uint8_t      letter;
    ir_nav_cmd_t cmd;
    uint8_t      packed;

    g_next.nav_cmd = IR_NAV_NONE;
    g_next.letter  = IR_LETTER_BAD;

    len    = ir_c39_decode_word(g_ctx.pats, g_ctx.chars, word, sizeof(word));
    cmd    = (len > 0u) ? ir_nav_command(word) : IR_NAV_NONE;
    letter = (len > 0u) ? ir_letter_code(word[0]) : IR_LETTER_BAD;

    if ((g_ctx.chars >= 3u)
        && (((IR_C39_STAR == g_ctx.pats[0])
             && (IR_C39_STAR == g_ctx.pats[g_ctx.chars - 1u]))
            || ((ir_c39_reverse(IR_C39_STAR) == g_ctx.pats[0])
                && (ir_c39_reverse(IR_C39_STAR)
                    == g_ctx.pats[g_ctx.chars - 1u]))))
    {
        packed = s3_nav_pack(letter, cmd);

        g_next.nav_cmd     = cmd & 0x07u;
        g_next.letter      = letter & 0x1Fu;
        g_next.barcode_seq = (g_next.barcode_seq + 1u) & 0x0Fu;

        // Never block the sensing loop: a full buffer drops the command.
        if (E_OK != tk_snd_mbf(mbf_s3_nav, &packed, S3_NAV_MSG_BYTES,
                               TMO_POL))
        {
            tm_printf((UB*)"ir: nav buffer full, command dropped\n");
        }
        s3_publish_barcode((len > 0u) ? word[0] : '?', (uint32_t) cmd);
        tm_printf((UB*)"ir: barcode \"%s\" cmd=%u\n", word,
                  (unsigned int) cmd);
    }
    else if (0u != g_ctx.chars)
    {
        // DEBUG: a read that did not decode. Show it on the dashboard as
        // '?' so failed reads are visible without the serial port.
        s3_publish_barcode('?', (uint32_t) IR_NAV_NONE);
    }
    else
    {
        // Nothing complete was read (noise or a single bar).
    }

    // DEBUG: every read attempt, decoded or not. Printed after reading,
    // so it cannot disturb the 2 ms bar timing.
    {
        uint8_t idx;

        uint32_t reason = ir_scan_reason(len);

        tm_printf((UB*)"ir: barcode read %u ms, chars=%u elem=%u "
                       "narrow=%u ms -> %s (%s)\n",
                  (unsigned int) (now - g_ctx.state_ms),
                  (unsigned int) g_ctx.chars,
                  (unsigned int) g_ctx.elem,
                  (unsigned int) g_ctx.min_elem_ms,
                  (len > 0u) ? word : "FAIL",
                  g_scan_reason_text[reason]);
        s3_publish_scan(reason);
        for (idx = 0u; idx < g_ctx.chars; idx++)
        {
            tm_printf((UB*)"ir:   char %u pattern 0x%03x '%c'\n",
                      (unsigned int) idx,
                      (unsigned int) g_ctx.pats[idx],
                      ir_c39_lookup(g_ctx.pats[idx]));
        }
    }

    ir_set_state(IR_TRACK, now);
    return;
}

static void
ir_handle_barcode (uint32_t now)
{
    if (g_b_lift_override)
    {
        ir_set_line_pos(&g_next, 0);
        g_next.sensors = 0u;
        g_next.junction = IR_JUNC_NONE;
        g_next.lost_timeout = 1u;
        ir_set_state(IR_LINE_LOST, now);
        g_ctx.barcode_run = 0u;
        return;
    }

    uint8_t  bits = ir_read_barcode_sensor();

    // Keep the line (C/R) position live every IR_TRACK_MS while reading,
    // so the VC can still steer. The barcode sensor is on the LEFT: on a
    // left bend the line itself slides under it, which started a "barcode"
    // and froze the line position, so the car drove straight off every
    // left bend (video 25 Nov 17:44).
    {
        static uint8_t s_line_div = 0u;

        s_line_div++;
        if (s_line_div >= (uint8_t) (IR_TRACK_MS / IR_BARCODE_MS))
        {
            uint16_t dark[2];
            int16_t  pos;

            s_line_div = 0u;
            g_next.sensors = ir_read_line_sensors(dark);
            if (ir_line_position_lr(dark[0], dark[1], &pos))
            {
                ir_set_line_pos(&g_next, pos);
                g_ctx.last_line_pos = pos;
            }
            else
            {
                ir_set_line_pos(&g_next, g_ctx.last_line_pos);
            }
        }
    }
#if IR_PEAK_EDGES
    bool     b_black = ir_peak_update(ir_bar_lift());
    (void) bits;
#else
    bool     b_black = (0u != (bits & IR_SENS_LEFT));
#endif
    uint32_t span = now - g_ctx.edge_ms;
    uint32_t quiet;
    uint32_t black_max = IR_BLACK_MAX_MS;
    uint32_t edge = now;            // Time of an accepted colour change.

    // Debounce: a change counts only once it has lasted IR_DEBOUNCE_MS,
    // and is then dated from its first sample, so widths stay accurate.
    if (b_black != (0u != g_ctx.b_black))
    {
        if (!g_b_edge_pending)
        {
            g_b_edge_pending  = true;
            g_edge_pending_ms = now;
        }
        if ((now - g_edge_pending_ms) < IR_DEBOUNCE_MS)
        {
            b_black = (0u != g_ctx.b_black);    // Not yet: no change.
        }
        else
        {
            edge = g_edge_pending_ms;
            span = edge - g_ctx.edge_ms;
            g_b_edge_pending = false;
        }
    }
    else
    {
        g_b_edge_pending = false;               // Flicker: ignored.
    }

    if (b_black != (0u != g_ctx.b_black))
    {
        uint16_t width = (span > 0xFFFFu) ? 0xFFFFu : (uint16_t) span;

        if (g_scan_n < PROTO_SC_MAX_ELEMS)  // Every element, gaps too.
        {
            g_scan_ms[g_scan_n] = width;
            g_scan_n++;
        }

        if (0u != g_ctx.b_gap)
        {
            // End of the narrow gap between two characters: not data.
            g_ctx.b_gap = 0u;
            g_ctx.elem  = 0u;
        }
        else
        {
            g_ctx.widths[g_ctx.elem] = width;
            // The very first bar is still only known to within one 10 ms
            // track tick, so it does not set the narrow-width estimate.
            if (((0u != g_ctx.chars) || (0u != g_ctx.elem))
                && (width < g_ctx.min_elem_ms))
            {
                g_ctx.min_elem_ms = width;
            }
            if (width > g_ctx.max_elem_ms)
            {
                g_ctx.max_elem_ms = width;
            }
            g_ctx.elem++;

            if (g_ctx.elem >= IR_C39_ELEMENTS)
            {
                if (g_ctx.chars < IR_MAX_CHARS)
                {
                    uint16_t pat = ir_c39_pattern(g_ctx.widths);
                    uint8_t  k;

                    g_ctx.pats[g_ctx.chars] = pat;
                    // Narrow widths, for the "uneven speed" flag. The first
                    // bar of the scan is only known to 10 ms: skipped.
                    for (k = (0u == g_ctx.chars) ? 1u : 0u;
                         (0u != pat) && (k < IR_C39_ELEMENTS); k++)
                    {
                        if (0u == (pat & (1u << (IR_C39_ELEMENTS - 1u - k))))
                        {
                            if (g_ctx.widths[k] < g_scan_narrow_min)
                            {
                                g_scan_narrow_min = g_ctx.widths[k];
                            }
                            if (g_ctx.widths[k] > g_scan_narrow_max)
                            {
                                g_scan_narrow_max = g_ctx.widths[k];
                            }
                        }
                    }
                    g_ctx.chars++;
                }
                g_ctx.elem  = 0u;
                g_ctx.b_gap = 1u;

                // Stop '*' read (start '*', a letter, end '*', either
                // direction): the word is complete, finish now instead of
                // waiting for the white quiet zone.
                if ((g_ctx.chars >= 3u)
                    && (((IR_C39_STAR == g_ctx.pats[0])
                         && (IR_C39_STAR == g_ctx.pats[g_ctx.chars - 1u]))
                        || ((ir_c39_reverse(IR_C39_STAR) == g_ctx.pats[0])
                            && (ir_c39_reverse(IR_C39_STAR)
                                == g_ctx.pats[g_ctx.chars - 1u]))))
                {
                    g_ctx.b_black = b_black ? 1u : 0u;
                    g_ctx.edge_ms = edge;
                    ir_barcode_finish(now);
                    return;
                }
            }
        }

        g_ctx.b_black = b_black ? 1u : 0u;
        g_ctx.edge_ms = edge;
        return;
    }

    // No transition this sample.
    // End of barcode: white for longer than any element can be.
    // The first bar of '*' is narrow and the next space is wide (about
    // 3 narrow), so 4 x the widest so far never cuts a real space short.
    quiet = IR_QUIET_WIDEST * (uint32_t) g_ctx.max_elem_ms;
    if (quiet < IR_QUIET_MIN_MS)
    {
        quiet = IR_QUIET_MIN_MS;
    }
    else if (quiet > IR_QUIET_MAX_MS)
    {
        quiet = IR_QUIET_MAX_MS;
    }
    else
    {
        // In range.
    }

    // Mid-word (a character started, stop '*' not read yet): a slow or
    // paused car makes one space very long, and the short limit above cut
    // the word in half and restarted from the middle. Only a lone bar
    // (noise, a junction bar) keeps the short limit, so line tracking is
    // not held up by it.
    if ((0u != g_ctx.chars) || (g_ctx.elem > 1u))
    {
        quiet = IR_QUIET_MAX_MS;

        // Same for black: a fixed 1 s cut slow reads off at a wide bar
        // (test 25 Nov: 11- and 13-element reads ended "solid black" at
        // 1020 ms). Mid-word, black only counts as a pad/junction when it
        // is 4 x longer than the widest element so far (1-2.5 s).
        black_max = IR_QUIET_WIDEST * (uint32_t) g_ctx.max_elem_ms;
        if (black_max < IR_BLACK_MAX_MS)
        {
            black_max = IR_BLACK_MAX_MS;
        }
        else if (black_max > IR_QUIET_MAX_MS)
        {
            black_max = IR_QUIET_MAX_MS;
        }
        else
        {
            // In range.
        }
    }

    if ((!b_black) && (span > quiet))
    {
        ir_barcode_finish(now);             // White quiet zone: done.
    }
    else if (b_black && (span > black_max))
    {
        if (g_scan_n < PROTO_SC_MAX_ELEMS)
        {
            g_scan_ms[g_scan_n] = (uint16_t) span;
            g_scan_n++;
        }
        s3_publish_scan(PROTO_SCAN_SOLID);
        g_ctx.junc      = IR_JUNC_BOTH;     // Solid black: T or cross.
        g_ctx.junc_ms   = now;
        g_ctx.b_armed   = 0u;               // Not again until white.
        g_next.junction = IR_JUNC_BOTH;
        ir_set_state(IR_TRACK, now);
    }
    else if ((now - g_ctx.state_ms) > IR_BARCODE_MAX_MS)
    {
        g_b_scan_timeout = true;
        ir_barcode_finish(now);             // Safety net.
    }
    else
    {
        // Still reading.
    }

    return;
}

static void
ir_handle_lost (uint32_t now)
{
    if (g_b_lift_override)
    {
        ir_set_line_pos(&g_next, 0);
        g_next.sensors = 0u;
        g_next.junction = IR_JUNC_NONE;
        g_next.lost_timeout = 1u;
        ir_set_state(IR_LINE_LOST, now);
        return;
    }

    uint16_t line_dark[2];
    uint8_t line_bits = ir_read_line_sensors(line_dark);
    uint8_t barcode = ir_read_barcode_sensor();
    uint8_t bits = line_bits;

    g_next.sensors = line_bits;
    ir_set_line_pos(&g_next, g_ctx.last_line_pos);
    g_next.junction = IR_JUNC_NONE;

    if ((0u != (barcode & IR_SENS_LEFT))
        && (!g_b_steer_locked))
    {
        g_next.lost_timeout = 0u;
        if (0u == g_ctx.barcode_run)
        {
            g_ctx.bar_first_ms = now;
        }
        if (g_ctx.barcode_run < IR_BARCODE_CONFIRM_SAMPLES)
        {
            g_ctx.barcode_run++;
        }
        if (g_ctx.barcode_run >= IR_BARCODE_CONFIRM_SAMPLES)
        {
            ir_barcode_begin(now, g_ctx.bar_first_ms);
            g_ctx.barcode_run = 0u;
        }
    }
    else
    {
        g_ctx.barcode_run = 0u;
    }

    if (0u != bits)
    {
        g_next.lost_timeout = 0u;
        ir_set_state(IR_TRACK, now);
    }
    else if (((now - g_ctx.state_ms) > IR_LOST_TIMEOUT_MS)
             && (!g_b_steer_locked))
    {
        g_next.lost_timeout = 1u;           // Reported, VC decides.
    }
    else
    {
        // Keep searching.
    }

    return;
}

static void
ir_publish_line (uint32_t now)
{
    uint8_t      frame[PROTO_MAX_FRAME_BYTES];
    proto_bits_t bits;
    uint32_t     age = now - g_ctx.last_pub_ms;
    bool         b_event = (g_next.state != g_last_pub_state)
                           || (g_next.junction != g_last_pub_junc)
                           || (g_next.sensors != g_last_pub_sensors);

    // Publish at once on a state / sensor / junction change. A line_pos
    // that only wobbles goes out at most every IR_POS_PUB_MS, and the
    // frame is refreshed every IR_LINE_PUB_MS so the panel stays live.
    if (!b_event
        && !((ir_line_pos(&g_next) != g_last_pub_pos) && (age >= IR_POS_PUB_MS))
        && (age < IR_LINE_PUB_MS))
    {
        return;
    }

    proto_init(&bits, frame, sizeof(frame));
    proto_begin(&bits, PROTO_TYPE_LINE);
    proto_put(&bits, g_next.state, PROTO_LN_STATE_BITS);
    proto_put(&bits, g_next.sensors, PROTO_LN_SENSORS_BITS);
    proto_put(&bits, g_next.junction, PROTO_LN_JUNCTION_BITS);
    proto_put_s(&bits, ir_line_pos(&g_next), PROTO_LN_POS_BITS);
    (void) s1_publish(frame, proto_end(&bits), false);

    // Console: only real changes, never the line_pos wobble.
    if (b_event)
    {
        tm_printf((UB*)"ir: state=%u sensors=%u junction=%u line_pos=%d lost=%u\n",
                  (unsigned int)g_next.state,
                  (unsigned int)g_next.sensors,
                  (unsigned int)g_next.junction,
                  (int)ir_line_pos(&g_next),
                  (unsigned int)g_next.lost_timeout);
    }

    g_last_pub_state   = (uint8_t) g_next.state;
    g_last_pub_junc    = (uint8_t) g_next.junction;
    g_last_pub_sensors = (uint8_t) g_next.sensors;
    g_last_pub_pos     = (int16_t) ir_line_pos(&g_next);
    g_ctx.last_pub_ms = now;

    return;
}

// Sweep calibration requests from the VC (other core): flags, acted on here.
static void
ir_sweep_requests (uint32_t now)
{
    static char const names[3] = {'L', 'C', 'R'};
    uint8_t idx;

    if (g_b_sweep_begin)
    {
        g_b_sweep_begin = false;
        for (idx = 0u; idx < 3u; idx++)
        {
            g_ctx.cal_saved[idx] = g_ctx.cal[idx];
            g_ctx.sw_lo[idx]     = IR_ADC_MAX;
            g_ctx.sw_hi[idx]     = 0u;
        }
        g_ctx.b_sweeping  = true;
        g_ctx.barcode_run = 0u;
        g_ctx.b_armed     = 1u;
        tm_printf((UB*)"ir: sweep calibration started\n");
        ir_set_state(IR_CALI, now);
        ir_publish_line(now);
    }

    if (g_b_sweep_cancel)
    {
        g_b_sweep_cancel = false;
        g_b_sweep_end    = false;
        if (g_ctx.b_sweeping)
        {
            for (idx = 0u; idx < 3u; idx++)
            {
                g_ctx.cal[idx] = g_ctx.cal_saved[idx];
            }
            g_ctx.b_sweeping = false;
            tm_printf((UB*)"ir: sweep calibration cancelled, old levels kept\n");
            s3_publish_ircal(2u, 0u);
            ir_set_state(IR_TRACK, now);
        }
    }

    if (g_b_sweep_end)
    {
        g_b_sweep_end = false;
        if (g_ctx.b_sweeping)
        {
            uint32_t ok_mask = 0u;

            for (idx = 0u; idx < 3u; idx++)
            {
                uint16_t lo = g_ctx.sw_lo[idx];
                uint16_t hi = g_ctx.sw_hi[idx];

                if ((hi > lo) && ((uint32_t) (hi - lo) >= IR_MIN_CONTRAST))
                {
                    g_ctx.cal[idx].white = lo;
                    g_ctx.cal[idx].black = hi;
                    ok_mask |= (1u << idx);
                    tm_printf((UB*)"ir: %c white=%u black=%u\n",
                              names[idx], (unsigned int) lo, (unsigned int) hi);
                }
                else
                {
                    // Never saw both colours: keep what it had.
                    tm_printf((UB*)"ir: %c low contrast (%u..%u), kept white=%u black=%u\n",
                              names[idx], (unsigned int) lo, (unsigned int) hi,
                              (unsigned int) g_ctx.cal[idx].white,
                              (unsigned int) g_ctx.cal[idx].black);
                }
            }
            // The barcode sensor (L) never crosses the line during the
            // sweep, so it never sees black and used to keep the 500/2000
            // defaults: its real black (~1500) then barely passed the 60 %
            // threshold and narrow bars were missed (test 25 Nov). Use its
            // own white and the middle sensor's black (same sensor type).
            if ((0u == (ok_mask & 1u)) && (0u != (ok_mask & 2u))
                && (g_ctx.sw_lo[0] < g_ctx.cal[1].black)
                && (((uint32_t) g_ctx.cal[1].black - g_ctx.sw_lo[0])
                    >= IR_MIN_CONTRAST))
            {
                g_ctx.cal[0].white = g_ctx.sw_lo[0];
                g_ctx.cal[0].black = g_ctx.cal[1].black;
                ok_mask |= 1u;
                tm_printf((UB*)"ir: L (barcode) uses own white=%u, "
                               "centre black=%u\n",
                          (unsigned int) g_ctx.cal[0].white,
                          (unsigned int) g_ctx.cal[0].black);
            }
            g_ctx.b_sweeping = false;
            tm_printf((UB*)"ir: sweep calibration done\n");
            s3_publish_ircal(1u, ok_mask);
            ir_set_state(IR_TRACK, now);
        }
    }

    return;
}

static void
ir_sensor_task (INT stacd, void * exinf)
{
    (void) stacd;
    (void) exinf;

    tm_printf((UB*)"ir: started, calibrating (keep the robot on the line)\n");
    tm_printf((UB*)"ir: DEBUG first ADC read, CS=0x%08x\n",
              (unsigned int) (*(volatile UW *) 0x4004C000u));

    for (;;)
    {
        uint32_t now = ir_now_ms();
        uint8_t  state;

        IR_STAGE(1u);
        if (g_b_recal_request)
        {
            g_b_recal_request = false;
            g_ctx.cal_count   = 0u;
            g_ctx.barcode_run = 0u;
            g_ctx.b_armed     = 1u;
            tm_printf((UB*)"ir: recalibrating (keep the robot on the line)\n");
            ir_set_state(IR_CALI, now);
            ir_publish_line(now);       // Dashboard sees CALI at once.
        }
        IR_STAGE(9u);
        ir_sweep_requests(now);
        state = (uint8_t) (g_next.state & IR_MASK);

        IR_STAGE(2u);
        g_handlers[state](now);

        // DEBUG: raw sensor readings twice a second, never while a
        // barcode is being timed. Barcode sensor (L) is black when its
        // dark value >= 600 and white when <= 400.
        if ((IR_BARCODE != (g_next.state & IR_MASK))
            && ((now - g_dbg_ms) >= IR_DBG_RAW_MS))
        {
            g_dbg_ms = now;
            tm_printf((UB*)"ir: RAW L=%u C=%u R=%u | cal white=%u black=%u"
                           " | L dark=%u (%s)\n",
                      (unsigned int) g_dbg_raw[0],
                      (unsigned int) g_dbg_raw[1],
                      (unsigned int) g_dbg_raw[2],
                      (unsigned int) g_ctx.cal[0].white,
                      (unsigned int) g_ctx.cal[0].black,
                      (unsigned int) ir_darkness(g_dbg_raw[0], &g_ctx.cal[0]),
                      ir_is_black(g_dbg_raw[0], false, &g_ctx.cal[0])
                          ? "BLACK" : "white");
        }

        // Publish the tick's result to readers in one locked copy.
        g_next.timestamp_ms = now;
        IR_STAGE(5u);
        {
            ER mer = tk_loc_mtx(g_ir_mutex, TMO_FEVR);

            IR_STAGE(6u);
            if (E_OK == mer)
            {
                g_snap = g_next;
                (void) tk_unl_mtx(g_ir_mutex);
            }
            else
            {
                static uint8_t s_mtx_prints = 0u;

                if (s_mtx_prints < 3u)
                {
                    s_mtx_prints++;
                    tm_printf((UB*)"ir: DEBUG tk_loc_mtx failed %d\n", (int) mer);
                }
            }
        }
        IR_STAGE(7u);

        // Not while reading a barcode: a print or publish on every sensor
        // edge in the 2 ms loop stretched the very widths being measured.
        // The first TRACK tick after the barcode publishes the change.
        if ((IR_CALI != g_next.state) && (IR_BARCODE != g_next.state))
        {
            ir_publish_line(now);
            ir_scan_tx_poll();          // One SCAN frame per tick.
        }

        IR_STAGE(8u);
        (void) tk_dly_tsk((IR_BARCODE == g_next.state) ? IR_BARCODE_MS
                                                        : IR_TRACK_MS);
    }
}

uint32_t
ir_debug_stage (void)
{
    return (g_ir_stage);
}

ER
ir_sense_init (void)
{
    T_CMTX cmtx = {0};
    T_CMBF cmbf = {0};

    // Priority inheritance: the VC (priority 10) may wait on this mutex.
    cmtx.mtxatr = TA_TFIFO | TA_INHERIT;
    g_ir_mutex  = tk_cre_mtx(&cmtx);
    if (g_ir_mutex < E_OK)
    {
        return (g_ir_mutex);
    }

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = S3_NAV_BUF_BYTES;
    cmbf.maxmsz = S3_NAV_MSG_BYTES;
    mbf_s3_nav  = tk_cre_mbf(&cmbf);
    if (mbf_s3_nav < E_OK)
    {
        return (mbf_s3_nav);
    }

    g_adc_dd = tk_opn_dev((UB const *) "adca", TD_READ);
    if (g_adc_dd < E_OK)
    {
        tm_printf((UB*)"ir: cannot open ADC: %d\n", (int) g_adc_dd);
        return (g_adc_dd);
    }

    ir_cal_set(&g_ctx.cal[0], 0u, 0u);  // Defaults until calibrated.
    g_ctx.cal[1] = g_ctx.cal[0];
    g_ctx.cal[2] = g_ctx.cal[0];
    g_ctx.b_armed       = 1u;
    g_ctx.last_line_pos = 0;
    g_next.state        = IR_CALI;
    ir_set_line_pos(&g_next, 0);
    g_next.letter       = IR_LETTER_BAD;
    g_next.nav_cmd      = IR_NAV_NONE;
    g_snap              = g_next;
    g_b_ready           = true;

    return (E_OK);
}

ER
ir_sense_start (void)
{
    ID tid;

    if (!g_b_ready)
    {
        return (E_OBJ);
    }

    // Core 0, like comms; VC and motion control are on core 1.
    tid = app_start_task(ir_sensor_task, S3_PRI_IR, TP_PRC1,
                         "ir_sensor_task");

    return ((tid < E_OK) ? tid : E_OK);
}

void
ir_lock_steering (bool b_lock)
{
    // One byte, one writer at a time (the VC): no lock needed.
    g_b_steer_locked = b_lock;

    return;
}

void
ir_set_lift_override (bool b_lifted)
{
    g_b_lift_override = b_lifted;
}

void
ir_request_calibration (void)
{
    g_b_recal_request = true;
}

void
ir_cal_sweep_begin (void)
{
    g_b_sweep_cancel = false;
    g_b_sweep_end    = false;
    g_b_sweep_begin  = true;
}

void
ir_cal_sweep_end (void)
{
    g_b_sweep_end = true;
}

void
ir_cal_sweep_cancel (void)
{
    g_b_sweep_cancel = true;
}

bool
ir_receive_nav_event (ir_nav_event_t * p_out)
{
    uint8_t packed;

    if ((NULL == p_out) || (mbf_s3_nav < E_OK))
    {
        return false;
    }

    if (tk_rcv_mbf(mbf_s3_nav, &packed, TMO_POL) == S3_NAV_MSG_BYTES)
    {
        ir_snapshot_t snap;
        p_out->letter = s3_nav_letter_of(packed);
        p_out->cmd = s3_nav_cmd_of(packed);

        ir_get_telemetry(&snap);
        p_out->barcode_seq = snap.barcode_seq;
        return true;
    }

    return false;
}
void
ir_get_telemetry (ir_snapshot_t * p_out)
{
    if ((NULL == p_out) || (g_ir_mutex < E_OK))
    {
        return;
    }

    if (E_OK == tk_loc_mtx(g_ir_mutex, TMO_FEVR))
    {
        *p_out = g_snap;
        (void) tk_unl_mtx(g_ir_mutex);
    }

    return;
}

/*** end of file ***/
