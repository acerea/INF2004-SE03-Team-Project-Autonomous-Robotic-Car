/** @file ir_algo.h
 *
 * @brief Subsystem 3: pure line-sensing and Code 39 barcode algorithms.
 *
 * @par
 * No kernel calls and no I/O, so every function can be unit-tested with
 * synthetic data, on the Pico (subsystem3_test.c) or on a PC.
 * The task that uses them is in ir_sense.c.
 *
 * Conventions:
 *  - Raw IR readings are 12-bit ADC codes. BLACK reads HIGH, white low.
 *  - "Darkness" is a reading normalised to 0 (white) .. 1000 (black)
 *    with the calibrated white and black levels.
 *  - Line position is -1000 (line under the LEFT sensor) .. +1000 (under
 *    the RIGHT sensor); 0 = centred. It needs 11 bits signed.
 *  - Code 39: 9 elements per character (bar, space, ... bar), exactly 3
 *    of them wide; characters are separated by a narrow gap.
 */
#ifndef IR_ALGO_H
#define IR_ALGO_H

#include <stdint.h>
#include <stdbool.h>

#define IR_ADC_MAX              (4095u)     // 12-bit ADC.
#define IR_DARK_MAX             (1000u)     // Darkness full scale.
#define IR_POS_MAX              (1000)      // |line position| maximum.
#define IR_POS_BITS             (11u)       // -1024..1023 holds +/-1000.
#define IR_LOST_DARK_SUM        (300u)      // Sum of 3 darkness below = no line.

// Calibration defaults (used if the start-up calibration has too little
// contrast). They are the values the original code used.
#define IR_DEFAULT_WHITE        (500u)
#define IR_DEFAULT_BLACK        (2000u)
#define IR_MIN_CONTRAST         (600u)      // black - white needed to calibrate.

// Hysteresis: black above 60 % of the contrast, white below 40 %.
#define IR_BLACK_PCT            (60u)
#define IR_WHITE_PCT            (40u)

// Code 39.
#define IR_C39_ELEMENTS         (9u)
#define IR_C39_WIDE_COUNT       (3u)
#define IR_C39_PATTERN_MASK     (0x1FFu)    // 9 bits.
#define IR_C39_STAR             (0x094u)    // '*' = 010010100 (start/stop).
#define IR_C39_MIN_RATIO_X10    (15u)       // 3rd widest >= 1.5 x 4th widest.
#define IR_C39_BAD              ('?')

// Navigation commands: same values as PROTO_BC_CMD_xxx (wire_format.h).
typedef uint8_t ir_nav_cmd_t;               // 3 bits.
#define IR_NAV_NONE             (0u)
#define IR_NAV_LEFT             (1u)
#define IR_NAV_RIGHT            (2u)
#define IR_NAV_STRAIGHT         (3u)
#define IR_NAV_UTURN            (4u)

#define IR_LETTER_BAD           (31u)       // 5-bit letter code: unreadable.

// Calibrated levels, 12-bit ADC codes.
typedef struct
{
    uint16_t white;
    uint16_t black;
} ir_cal_t;

void         ir_cal_set(ir_cal_t * p_cal, uint16_t lo, uint16_t hi);
uint16_t     ir_darkness(uint16_t raw, ir_cal_t const * p_cal);
bool         ir_is_black(uint16_t raw, bool b_was_black,
                         ir_cal_t const * p_cal);
bool         ir_line_position(uint16_t dark_l, uint16_t dark_c,
                              uint16_t dark_r, int16_t * p_pos);

uint16_t     ir_c39_pattern(uint16_t const p_widths[IR_C39_ELEMENTS]);
uint16_t     ir_c39_reverse(uint16_t pattern);
char         ir_c39_lookup(uint16_t pattern);
uint8_t      ir_c39_decode_word(uint16_t const * p_pats, uint8_t n_pats,
                                char * p_out, uint8_t out_cap);
ir_nav_cmd_t ir_nav_command(char const * p_word);
uint8_t      ir_letter_code(char letter);

#endif /* IR_ALGO_H */

/*** end of file ***/