/** @file ir_algo.c
 *
 * @brief Subsystem 3: pure line-sensing and Code 39 barcode algorithms.
 *
 * @par
 * See ir_algo.h for the conventions. No kernel calls in this file.
 */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "ir_algo.h"

// Code 39 letters A-Z and '*'. Pattern bit 8 = first element (a bar),
// 1 = wide. Kept in the .c file (Barr-C 4.2: no definitions in headers).
typedef struct
{
    uint16_t pattern;
    char     letter;
} ir_c39_entry_t;

static ir_c39_entry_t const g_c39_table[] =
{
    {0x109u, 'A'}, {0x049u, 'B'}, {0x148u, 'C'}, {0x019u, 'D'},
    {0x118u, 'E'}, {0x058u, 'F'}, {0x00Du, 'G'}, {0x10Cu, 'H'},
    {0x04Cu, 'I'}, {0x01Cu, 'J'}, {0x103u, 'K'}, {0x043u, 'L'},
    {0x142u, 'M'}, {0x013u, 'N'}, {0x112u, 'O'}, {0x052u, 'P'},
    {0x007u, 'Q'}, {0x106u, 'R'}, {0x046u, 'S'}, {0x016u, 'T'},
    {0x181u, 'U'}, {0x0C1u, 'V'}, {0x1C0u, 'W'}, {0x091u, 'X'},
    {0x190u, 'Y'}, {0x0D0u, 'Z'}, {IR_C39_STAR, '*'},
};

#define IR_C39_TABLE_SIZE  (sizeof(g_c39_table) / sizeof(g_c39_table[0]))

// Barcode word -> navigation command. Single letters follow the example
// in the project write-up (A left, B right, C straight, D U-turn).
// NOTE: Update this table when the real course barcodes are known.
typedef struct
{
    char const * p_word;
    ir_nav_cmd_t cmd;
} ir_nav_entry_t;

static ir_nav_entry_t const g_nav_table[] =
{
    {"A", IR_NAV_LEFT},  {"B", IR_NAV_RIGHT},
    {"C", IR_NAV_STRAIGHT}, {"D", IR_NAV_UTURN},
    {"LEFT", IR_NAV_LEFT}, {"RIGHT", IR_NAV_RIGHT},
    {"STRAIGHT", IR_NAV_STRAIGHT}, {"UTURN", IR_NAV_UTURN},
};

#define IR_NAV_TABLE_SIZE  (sizeof(g_nav_table) / sizeof(g_nav_table[0]))

static bool
str_equal (char const * p_a, char const * p_b)
{
    while ((*p_a != '\0') && (*p_a == *p_b))
    {
        p_a++;
        p_b++;
    }

    return (*p_a == *p_b);
}

void
ir_cal_set (ir_cal_t * p_cal, uint16_t lo, uint16_t hi)
{
    if ((hi > lo) && ((uint32_t) (hi - lo) >= IR_MIN_CONTRAST))
    {
        p_cal->white = lo;
        p_cal->black = hi;
    }
    else
    {
        p_cal->white = IR_DEFAULT_WHITE;
        p_cal->black = IR_DEFAULT_BLACK;
    }

    return;
}

uint16_t
ir_darkness (uint16_t raw, ir_cal_t const * p_cal)
{
    uint32_t dark = 0u;

    if (raw >= p_cal->black)
    {
        dark = IR_DARK_MAX;
    }
    else if (raw > p_cal->white)
    {
        dark = ((uint32_t) (raw - p_cal->white) * IR_DARK_MAX)
               / (uint32_t) (p_cal->black - p_cal->white);
    }
    else
    {
        dark = 0u;      // At or below white.
    }

    return ((uint16_t) dark);
}

bool
ir_is_black (uint16_t raw, bool b_was_black, ir_cal_t const * p_cal)
{
    uint16_t dark = ir_darkness(raw, p_cal);
    bool     b_black = b_was_black;     // Between the two: keep.

    if (dark >= ((IR_DARK_MAX * IR_BLACK_PCT) / 100u))
    {
        b_black = true;
    }
    else if (dark <= ((IR_DARK_MAX * IR_WHITE_PCT) / 100u))
    {
        b_black = false;
    }
    else
    {
        // Hysteresis band: no change.
    }

    return (b_black);
}

bool
ir_line_position (uint16_t dark_l, uint16_t dark_c, uint16_t dark_r,
                  int16_t * p_pos)
{
    uint32_t sum = (uint32_t) dark_l + dark_c + dark_r;
    bool     b_found = false;

    if (sum >= IR_LOST_DARK_SUM)
    {
        // Weighted mean of sensor positions -1, 0, +1, scaled to +/-1000.
        int32_t num = ((int32_t) dark_r - (int32_t) dark_l) * IR_POS_MAX;

        *p_pos  = (int16_t) (num / (int32_t) sum);
        b_found = true;
    }

    return (b_found);
}

uint16_t
ir_c39_pattern (uint16_t const p_widths[IR_C39_ELEMENTS])
{
    uint16_t pattern = 0u;
    uint16_t wide_min = 0xFFFFu;    // Narrowest of the 3 wide elements.
    uint16_t narrow_max = 0u;       // Widest of the 6 narrow elements.
    uint8_t  idx;
    uint8_t  pick;

    // Mark the 3 widest elements (Code 39 has exactly 3 wide ones). This
    // is independent of speed, unlike a fixed width threshold.
    for (pick = 0u; pick < IR_C39_WIDE_COUNT; pick++)
    {
        uint8_t  best = 0xFFu;
        uint16_t best_w = 0u;

        for (idx = 0u; idx < IR_C39_ELEMENTS; idx++)
        {
            uint16_t bit = (uint16_t) (1u << (IR_C39_ELEMENTS - 1u - idx));

            if ((0u == (pattern & bit)) && ((0xFFu == best)
                                            || (p_widths[idx] > best_w)))
            {
                best   = idx;
                best_w = p_widths[idx];
            }
        }

        pattern |= (uint16_t) (1u << (IR_C39_ELEMENTS - 1u - best));
        if (best_w < wide_min)
        {
            wide_min = best_w;
        }
    }

    for (idx = 0u; idx < IR_C39_ELEMENTS; idx++)
    {
        uint16_t bit = (uint16_t) (1u << (IR_C39_ELEMENTS - 1u - idx));

        if ((0u == (pattern & bit)) && (p_widths[idx] > narrow_max))
        {
            narrow_max = p_widths[idx];
        }
    }

    // Reject a character whose wide and narrow elements are too close.
    if (((uint32_t) wide_min * 10u)
        < ((uint32_t) narrow_max * IR_C39_MIN_RATIO_X10))
    {
        pattern = 0u;   // 0 is not a valid Code 39 pattern.
    }

    return (pattern);
}

uint16_t
ir_c39_reverse (uint16_t pattern)
{
    uint16_t reversed = 0u;
    uint8_t  idx;

    for (idx = 0u; idx < IR_C39_ELEMENTS; idx++)
    {
        if (0u != (pattern & (uint16_t) (1u << idx)))
        {
            reversed |= (uint16_t) (1u << (IR_C39_ELEMENTS - 1u - idx));
        }
    }

    return (reversed);
}

char
ir_c39_lookup (uint16_t pattern)
{
    char   letter = IR_C39_BAD;
    size_t idx;

    for (idx = 0u; idx < IR_C39_TABLE_SIZE; idx++)
    {
        if (g_c39_table[idx].pattern == pattern)
        {
            letter = g_c39_table[idx].letter;
            break;
        }
    }

    return (letter);
}

uint8_t
ir_c39_decode_word (uint16_t const * p_pats, uint8_t n_pats, char * p_out,
                    uint8_t out_cap)
{
    bool    b_reverse = false;
    uint8_t len = 0u;
    uint8_t idx;

    if ((0u == n_pats) || (0u == out_cap))
    {
        return (0u);
    }

    // Direction: a forward read starts with '*'; a backward read starts
    // with '*' reversed (which looks like 'P'). Without start/stop
    // characters, pick the direction in which more characters decode.
    if (IR_C39_STAR == p_pats[0])
    {
        b_reverse = false;
    }
    else if (ir_c39_reverse(IR_C39_STAR) == p_pats[0])
    {
        b_reverse = true;
    }
    else
    {
        uint8_t fwd_ok = 0u;
        uint8_t rev_ok = 0u;

        for (idx = 0u; idx < n_pats; idx++)
        {
            if (IR_C39_BAD != ir_c39_lookup(p_pats[idx]))
            {
                fwd_ok++;
            }
            if (IR_C39_BAD != ir_c39_lookup(ir_c39_reverse(p_pats[idx])))
            {
                rev_ok++;
            }
        }
        b_reverse = (rev_ok > fwd_ok);
    }

    for (idx = 0u; idx < n_pats; idx++)
    {
        uint16_t pat = b_reverse ? ir_c39_reverse(p_pats[n_pats - 1u - idx])
                                 : p_pats[idx];
        char     letter = ir_c39_lookup(pat);

        // '*' is framing only; keep everything else, '?' included, so a
        // bad read never matches a command.
        if (('*' != letter) && (len < (out_cap - 1u)))
        {
            p_out[len] = letter;
            len++;
        }
    }

    p_out[len] = '\0';
    return (len);
}

ir_nav_cmd_t
ir_nav_command (char const * p_word)
{
    ir_nav_cmd_t cmd = IR_NAV_NONE;
    size_t       idx;

    for (idx = 0u; idx < IR_NAV_TABLE_SIZE; idx++)
    {
        if (str_equal(p_word, g_nav_table[idx].p_word))
        {
            cmd = g_nav_table[idx].cmd;
            break;
        }
    }

    return (cmd);
}

uint8_t
ir_letter_code (char letter)
{
    uint8_t code = IR_LETTER_BAD;

    if ((letter >= 'A') && (letter <= 'Z'))
    {
        code = (uint8_t) (letter - 'A');
    }

    return (code);
}

/*** end of file ***/