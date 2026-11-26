/** @file ir_sense.h
 *
 * @brief Subsystem 3: IR line sensing and barcode reading, public API.
 *
 * @par
 * One task (ir_sensor_task) owns the ADC (GP26 left, GP27 centre, GP28
 * right). It calibrates, estimates the line position, detects line loss
 * and junctions, reads Code 39 barcodes, and reports:
 *   - a snapshot any task may read with ir_get_telemetry() (IF-14),
 *   - one byte per decoded barcode in mbf_s3_nav (IF-15, for the VC),
 *   - LINE and BARCODE telemetry frames through s1_publish() (IF-10).
 */
#ifndef IR_SENSE_H
#define IR_SENSE_H

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include "ir_algo.h"

// IR task state, 2 bits (change requirement 1).
#define IR_MASK                 (0x03u)
#define IR_CALI                 (0x00u)     // Calibrating at power-up.
#define IR_TRACK                (0x01u)     // Following the line.
#define IR_BARCODE              (0x02u)     // Reading a barcode.
#define IR_LINE_LOST            (0x03u)     // No sensor sees the line.

// Junction field, 2 bits.
#define IR_JUNC_NONE            (0u)
#define IR_JUNC_LEFT            (1u)        // Branch to the left.
#define IR_JUNC_RIGHT           (2u)        // Branch to the right.
#define IR_JUNC_BOTH            (3u)        // T / cross / solid black.

// Sensor bits (black = 1), 3 bits.
#define IR_SENS_LEFT            (0x04u)
#define IR_SENS_CENTRE          (0x02u)
#define IR_SENS_RIGHT           (0x01u)

// Snapshot of subsystem 3, read with ir_get_telemetry(). The fields
// share one 32-bit word, sized to their range (change requirement 1);
// it is written and read only under the subsystem's mutex.
typedef struct
{
    uint32_t     timestamp_ms;          // Time of the last update.
    unsigned int line_pos_enc : 11;     // Line position + 1000 (0..2000); use ir_line_pos().
    unsigned int state        : 2;      // IR_CALI .. IR_LINE_LOST.
    unsigned int sensors      : 3;      // IR_SENS_xxx bits.
    unsigned int junction     : 2;      // IR_JUNC_xxx.
    unsigned int lost_timeout : 1;      // Line lost for > IR_LOST_TIMEOUT_MS.
    unsigned int nav_cmd      : 3;      // Last barcode command, IR_NAV_xxx.
    unsigned int letter       : 5;      // Last barcode letter, 0-25 / 31.
    unsigned int barcode_seq  : 4;      // +1 per barcode read (wraps).
} ir_snapshot_t;

// The line position is -1000..+1000 (+ = line to the right). Bit-fields
// must not be signed (Barr-C 5.3.a), so it is stored offset by
// IR_LINE_POS_OFFSET in an unsigned 11-bit field (0..2000 fits 0..2047).
// Read and write it only through these two functions.
#define IR_LINE_POS_OFFSET      (1000)

static inline int16_t
ir_line_pos (ir_snapshot_t const * p_snap)
{
    return (int16_t) ((int32_t) p_snap->line_pos_enc - IR_LINE_POS_OFFSET);
}

static inline void
ir_set_line_pos (ir_snapshot_t * p_snap, int32_t pos)
{
    if (pos < -IR_LINE_POS_OFFSET)
    {
        pos = -IR_LINE_POS_OFFSET;
    }
    else if (pos > IR_LINE_POS_OFFSET)
    {
        pos = IR_LINE_POS_OFFSET;
    }
    else
    {
        // In range.
    }
    // NOTE: pos + offset is 0..2000 after the clamp, so the cast is safe.
    p_snap->line_pos_enc = (unsigned int) (pos + IR_LINE_POS_OFFSET);
}

typedef struct
{
    uint8_t     letter;
    ir_nav_cmd_t cmd;
    uint8_t     barcode_seq;
} ir_nav_event_t;
ER   ir_sense_init(void);
ER   ir_sense_start(void);
void ir_lock_steering(bool b_lock);
void ir_set_lift_override(bool b_lifted);

// Ask ir_sensor_task to run the start-up calibration again (robot on the
// line, not moving). Any task; acted on at the task's next tick.
void ir_request_calibration(void);

// Sweep calibration, driven by the VC while it turns the robot left and
// right over the line: begin -> every sensor records its own darkest and
// lightest reading -> end (keep the new levels) or cancel (keep the old).
// A sensor that saw too little contrast keeps its previous levels.
void ir_cal_sweep_begin(void);
void ir_cal_sweep_end(void);
void ir_cal_sweep_cancel(void);

// DEBUG: where ir_sensor_task is in its loop (see IR_STAGE_xxx in
// ir_sense.c), and the ADC channel it last asked for.
uint32_t ir_debug_stage(void);
void ir_get_telemetry(ir_snapshot_t * p_out);

// Non-blocking consumption of one decoded barcode command.
bool ir_receive_nav_event(ir_nav_event_t * p_out);

#endif /* IR_SENSE_H */

/*** end of file ***/
