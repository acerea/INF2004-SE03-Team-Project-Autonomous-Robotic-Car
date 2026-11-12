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

// ADC channels (TEAM.md: GP26 left, GP27 centre, GP28 right).
#define ADC_LEFT                (0)
#define ADC_CENTRE              (1)
#define ADC_RIGHT               (2)

#define IR_TRACK_MS             (10u)       // Line tracking period.
#define IR_BARCODE_MS           (2u)        // Barcode sampling period.
#define IR_CAL_SAMPLES          (50u)       // 0.5 s calibration.
#define IR_LOST_TIMEOUT_MS      (1000u)     // Line lost -> report.
#define IR_BLACK_MAX_MS         (1000u)     // Longer black: not a barcode.
#define IR_BARCODE_MAX_MS       (4000u)     // Give up on a barcode.
#define IR_QUIET_MIN_MS         (60u)       // Shortest end-of-barcode gap.
#define IR_QUIET_NARROWS        (4u)        // Quiet zone > 4 narrow widths
                                            // (wide <= 3, quiet >= 10).
#define IR_JUNC_SAMPLES         (3u)        // Junction must hold 30 ms.
#define IR_JUNC_HOLD_MS         (300u)      // Report a junction this long.
#define IR_SENS_SIDES           (IR_SENS_LEFT | IR_SENS_RIGHT)
#define IR_SENS_ALL             (IR_SENS_LEFT | IR_SENS_CENTRE | IR_SENS_RIGHT)
#define IR_LINE_PUB_MS          (1000u)     // LINE frame at least every 1 s.
#define IR_MAX_CHARS            (10u)       // '*' + 8 letters + '*'.
#define IR_WORD_MAX             (IR_MAX_CHARS + 1u)

// Decoder and tracker context, private to ir_sensor_task. Counters use
// the bits their range needs (change requirement 1).
typedef struct
{
    ir_cal_t cal;                       // Calibrated white / black.
    uint16_t cal_lo;                    // Calibration min / max.
    uint16_t cal_hi;
    uint32_t state_ms;                  // Time the state was entered.
    uint32_t edge_ms;                   // Last barcode transition.
    uint32_t last_pub_ms;               // Last LINE frame.
    uint32_t junc_ms;                   // Time the junction was seen.
    uint16_t widths[IR_C39_ELEMENTS];   // Element widths, ms.
    uint16_t pats[IR_MAX_CHARS];        // 9-bit character patterns.
    uint16_t min_elem_ms;               // Narrowest element so far.
    uint8_t  cal_count;                 // 0..50.
    uint8_t  elem    : 4;               // 0..9 elements of the character.
    uint8_t  chars   : 4;               // 0..10 characters read.
    uint8_t  b_black : 1;               // Centre sensor black (barcode).
    uint8_t  b_gap   : 1;               // In an inter-character gap.
    uint8_t  junc_run: 2;               // 0..3 junction samples.
    uint8_t  sensors : 3;               // Last IR_SENS_xxx bits.
    uint8_t  junc    : 2;               // Junction being reported.
    uint8_t  b_armed : 1;               // Sides saw white since last pad.
} ir_ctx_t;

typedef void (*ir_handler_t)(uint32_t now);

static ir_ctx_t       g_ctx;
static ir_snapshot_t  g_snap;           // Shared, under g_ir_mutex.
static ir_snapshot_t  g_next;           // Built by the task each tick.
static ID             g_ir_mutex = -1;
static ID             g_adc_dd = -1;
static bool           g_b_ready = false;
static uint8_t        g_last_pub_state = 0xFFu;
static uint8_t        g_last_pub_junc = 0xFFu;
static volatile bool  g_b_steer_locked = false;

EXPORT ID mbf_s3_nav;                   // IF-15.

static void ir_handle_cali(uint32_t now);
static void ir_handle_track(uint32_t now);
static void ir_handle_barcode(uint32_t now);
static void ir_handle_lost(uint32_t now);

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

    err = tk_srea_dev(g_adc_dd, channel, &value, 1, &asize);
    if ((err < E_OK) || (asize < 1))
    {
        value = 0u;     // Reads as white: a dead sensor looks like no line.
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

// Read the three sensors, classify them, update g_next.sensors.
static uint8_t
ir_read_sensors (uint16_t p_dark[3])
{
    uint16_t raw_l = ir_read_adc(ADC_LEFT);
    uint16_t raw_c = ir_read_adc(ADC_CENTRE);
    uint16_t raw_r = ir_read_adc(ADC_RIGHT);
    uint8_t  old = g_ctx.sensors;
    uint8_t  bits = 0u;

    if (ir_is_black(raw_l, 0u != (old & IR_SENS_LEFT), &g_ctx.cal))
    {
        bits |= IR_SENS_LEFT;
    }
    if (ir_is_black(raw_c, 0u != (old & IR_SENS_CENTRE), &g_ctx.cal))
    {
        bits |= IR_SENS_CENTRE;
    }
    if (ir_is_black(raw_r, 0u != (old & IR_SENS_RIGHT), &g_ctx.cal))
    {
        bits |= IR_SENS_RIGHT;
    }

    if (NULL != p_dark)
    {
        p_dark[0] = ir_darkness(raw_l, &g_ctx.cal);
        p_dark[1] = ir_darkness(raw_c, &g_ctx.cal);
        p_dark[2] = ir_darkness(raw_r, &g_ctx.cal);
    }

    g_ctx.sensors  = bits & 0x07u;
    g_next.sensors = bits & 0x07u;
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
        ir_cal_set(&g_ctx.cal, g_ctx.cal_lo, g_ctx.cal_hi);

        if ((g_ctx.cal.white != g_ctx.cal_lo)
            || (g_ctx.cal.black != g_ctx.cal_hi))
        {
            tm_printf((UB*)"ir: low contrast (%u..%u), default levels\n",
                      (unsigned int) g_ctx.cal_lo,
                      (unsigned int) g_ctx.cal_hi);
        }
        tm_printf((UB*)"ir: calibrated white=%u black=%u\n",
                  (unsigned int) g_ctx.cal.white,
                  (unsigned int) g_ctx.cal.black);
        ir_set_state(IR_TRACK, now);
    }

    return;
}

static void
ir_barcode_begin (uint32_t now)
{
    g_ctx.edge_ms     = now;
    g_ctx.elem        = 0u;
    g_ctx.chars       = 0u;
    g_ctx.b_black     = 1u;
    g_ctx.b_gap       = 0u;
    g_ctx.min_elem_ms = 0xFFFFu;
    ir_set_state(IR_BARCODE, now);

    return;
}

static void
ir_handle_track (uint32_t now)
{
    uint16_t dark[3];
    uint8_t  bits = ir_read_sensors(dark);
    uint8_t  junc = IR_JUNC_NONE;
    int16_t  pos = 0;

    if (IR_SENS_SIDES != (bits & IR_SENS_SIDES))
    {
        g_ctx.b_armed = 1u;             // Off any bar or pad.
    }

    // All three black: a barcode bar spans the track.
    if ((IR_SENS_ALL == bits) && (0u != g_ctx.b_armed)
        && (!g_b_steer_locked))
    {
        ir_barcode_begin(now);
        return;
    }

    if (0u == bits)
    {
        g_next.lost_timeout = 0u;
        ir_set_state(IR_LINE_LOST, now);
        return;
    }

    if (ir_line_position(dark[0], dark[1], dark[2], &pos))
    {
        g_next.line_pos = (signed int) pos;      // |pos| <= 1000: fits 11 bits.
    }

    // A side branch: centre and one side black, held for 30 ms.
    if (bits == (IR_SENS_LEFT | IR_SENS_CENTRE))
    {
        junc = IR_JUNC_LEFT;
    }
    else if (bits == (IR_SENS_RIGHT | IR_SENS_CENTRE))
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

static void
ir_barcode_finish (uint32_t now)
{
    char         word[IR_WORD_MAX];
    uint8_t      len;
    uint8_t      letter;
    ir_nav_cmd_t cmd;
    uint8_t      packed;

    len    = ir_c39_decode_word(g_ctx.pats, g_ctx.chars, word, sizeof(word));
    cmd    = (len > 0u) ? ir_nav_command(word) : IR_NAV_NONE;
    letter = (len > 0u) ? ir_letter_code(word[0]) : IR_LETTER_BAD;

    if (0u != g_ctx.chars)
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

    ir_set_state(IR_TRACK, now);
    return;
}

static void
ir_handle_barcode (uint32_t now)
{
    uint8_t  bits = ir_read_sensors(NULL);
    bool     b_black = (IR_SENS_SIDES == (bits & IR_SENS_SIDES));
    uint32_t span = now - g_ctx.edge_ms;
    uint32_t quiet;

    if (b_black != (0u != g_ctx.b_black))
    {
        uint16_t width = (span > 0xFFFFu) ? 0xFFFFu : (uint16_t) span;

        if (0u != g_ctx.b_gap)
        {
            // End of the narrow gap between two characters: not data.
            g_ctx.b_gap = 0u;
            g_ctx.elem  = 0u;
        }
        else
        {
            g_ctx.widths[g_ctx.elem] = width;
            g_ctx.elem++;
            if (width < g_ctx.min_elem_ms)
            {
                g_ctx.min_elem_ms = width;
            }

            if (g_ctx.elem >= IR_C39_ELEMENTS)
            {
                if (g_ctx.chars < IR_MAX_CHARS)
                {
                    g_ctx.pats[g_ctx.chars] = ir_c39_pattern(g_ctx.widths);
                    g_ctx.chars++;
                }
                g_ctx.elem  = 0u;
                g_ctx.b_gap = 1u;
            }
        }

        g_ctx.b_black = b_black ? 1u : 0u;
        g_ctx.edge_ms = now;
        return;
    }

    // No transition this sample.
    // End of barcode: white for longer than any element can be.
    quiet = IR_QUIET_NARROWS * (uint32_t) g_ctx.min_elem_ms;
    if (quiet < IR_QUIET_MIN_MS)
    {
        quiet = IR_QUIET_MIN_MS;
    }

    if ((!b_black) && (span > quiet))
    {
        ir_barcode_finish(now);             // White quiet zone: done.
    }
    else if (b_black && (span > IR_BLACK_MAX_MS))
    {
        g_ctx.junc      = IR_JUNC_BOTH;     // Solid black: T or cross.
        g_ctx.junc_ms   = now;
        g_ctx.b_armed   = 0u;               // Not again until white.
        g_next.junction = IR_JUNC_BOTH;
        ir_set_state(IR_TRACK, now);
    }
    else if ((now - g_ctx.state_ms) > IR_BARCODE_MAX_MS)
    {
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
    uint8_t bits = ir_read_sensors(NULL);

    if ((IR_SENS_ALL == bits) && (!g_b_steer_locked))
    {
        g_next.lost_timeout = 0u;       // A barcode with no line before it.
        ir_barcode_begin(now);
    }
    else if (0u != bits)
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

    if ((g_next.state == g_last_pub_state)
        && (g_next.junction == g_last_pub_junc)
        && ((now - g_ctx.last_pub_ms) < IR_LINE_PUB_MS))
    {
        return;
    }

    proto_init(&bits, frame, sizeof(frame));
    proto_begin(&bits, PROTO_TYPE_LINE);
    proto_put(&bits, g_next.state, PROTO_LN_STATE_BITS);
    proto_put(&bits, g_next.sensors, PROTO_LN_SENSORS_BITS);
    proto_put(&bits, g_next.junction, PROTO_LN_JUNCTION_BITS);
    proto_put_s(&bits, g_next.line_pos, PROTO_LN_POS_BITS);
    (void) s1_publish(frame, proto_end(&bits), false);

    g_last_pub_state  = (uint8_t) g_next.state;
    g_last_pub_junc   = (uint8_t) g_next.junction;
    g_ctx.last_pub_ms = now;

    return;
}

static void
ir_sensor_task (INT stacd, void * exinf)
{
    (void) stacd;
    (void) exinf;

    tm_printf((UB*)"ir: started, calibrating (keep the robot on the line)\n");

    for (;;)
    {
        uint32_t now = ir_now_ms();
        uint8_t  state = (uint8_t) (g_next.state & IR_MASK);

        g_handlers[state](now);

        // Publish the tick's result to readers in one locked copy.
        g_next.timestamp_ms = now;
        if (E_OK == tk_loc_mtx(g_ir_mutex, TMO_FEVR))
        {
            g_snap = g_next;
            (void) tk_unl_mtx(g_ir_mutex);
        }

        if (IR_CALI != g_next.state)
        {
            ir_publish_line(now);
        }

        (void) tk_dly_tsk((IR_BARCODE == g_next.state) ? IR_BARCODE_MS
                                                        : IR_TRACK_MS);
    }
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

    ir_cal_set(&g_ctx.cal, 0u, 0u);     // Defaults until calibrated.
    g_ctx.b_armed       = 1u;
    g_next.state        = IR_CALI;
    g_next.line_pos     = 0;
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