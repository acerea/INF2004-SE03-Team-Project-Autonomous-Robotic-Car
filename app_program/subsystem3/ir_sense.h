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
    signed int   line_pos     : 11;     // -1000..+1000, + = line to the right.
    unsigned int state        : 2;      // IR_CALI .. IR_LINE_LOST.
    unsigned int sensors      : 3;      // IR_SENS_xxx bits.
    unsigned int junction     : 2;      // IR_JUNC_xxx.
    unsigned int lost_timeout : 1;      // Line lost for > IR_LOST_TIMEOUT_MS.
    unsigned int nav_cmd      : 3;      // Last barcode command, IR_NAV_xxx.
    unsigned int letter       : 5;      // Last barcode letter, 0-25 / 31.
    unsigned int barcode_seq  : 4;      // +1 per barcode read (wraps).
} ir_snapshot_t;

ER   ir_sense_init(void);
ER   ir_sense_start(void);
void ir_lock_steering(bool b_lock);
void ir_get_telemetry(ir_snapshot_t * p_out);

#endif /* IR_SENSE_H */

/*** end of file ***/
