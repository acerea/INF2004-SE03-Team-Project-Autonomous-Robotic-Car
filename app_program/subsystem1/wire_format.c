/** @file wire_format.c
 *
 * @brief Bit-packed frame helpers shared by all subsystems.
 */
#include <stdint.h>
#include <stdbool.h>
#include "wire_format.h"

#define BITS_PER_BYTE   (8u)
#define MAX_FIELD_BITS  (24u)

static bool b_width_ok(uint8_t n_bits);

void
proto_init (proto_bits_t * p_bits, uint8_t * p_buf, uint32_t cap_bytes)
{
    p_bits->p_buf    = p_buf;
    p_bits->cap_bits = cap_bytes * BITS_PER_BYTE;
    p_bits->pos_bits = 0u;
    p_bits->b_error  = false;

    for (uint32_t idx = 0u; idx < cap_bytes; idx++)
    {
        p_buf[idx] = 0u;
    }

    return;
}

void
proto_open (proto_bits_t * p_bits, uint8_t const * p_buf, uint32_t len_bytes)
{
    // NOTE: Read-only use. proto_get never writes through p_buf.
    p_bits->p_buf    = (uint8_t *) p_buf;
    p_bits->cap_bits = len_bytes * BITS_PER_BYTE;
    p_bits->pos_bits = 0u;
    p_bits->b_error  = false;

    return;
}

void
proto_begin (proto_bits_t * p_bits, proto_type_t type)
{
    // NOTE: Seq is written as 0. The MQTT library patches byte 0.
    proto_put(p_bits, 0u, PROTO_SEQ_BITS);
    proto_put(p_bits, PROTO_VERSION, PROTO_VERSION_BITS);
    proto_put(p_bits, (uint32_t) type, PROTO_TYPE_BITS);

    return;
}

uint32_t
proto_end (proto_bits_t const * p_bits)
{
    return ((p_bits->pos_bits + (BITS_PER_BYTE - 1u)) / BITS_PER_BYTE);
}

void
proto_put (proto_bits_t * p_bits, uint32_t value, uint8_t n_bits)
{
    if (false == b_width_ok(n_bits))
    {
        p_bits->b_error = true;
    }
    else
    {
        // A value wider than its field is an error, never a silent wrap.
        if (0u != (value >> n_bits))
        {
            p_bits->b_error = true;
        }

        for (uint8_t idx = 0u; idx < n_bits; idx++)
        {
            if (p_bits->pos_bits >= p_bits->cap_bits)
            {
                p_bits->b_error = true;
            }
            else
            {
                uint32_t byte_idx = p_bits->pos_bits / BITS_PER_BYTE;
                uint32_t shift    = p_bits->pos_bits % BITS_PER_BYTE;

                if (0u != ((value >> idx) & 1u))
                {
                    // NOTE: Shift is below 8, so the result fits in 8 bits.
                    p_bits->p_buf[byte_idx] |= (uint8_t) (1u << shift);
                }

                p_bits->pos_bits++;
            }
        }
    }

    return;
}

void
proto_put_sat (proto_bits_t * p_bits, uint32_t value, uint8_t n_bits)
{
    if (false == b_width_ok(n_bits))
    {
        p_bits->b_error = true;
    }
    else
    {
        uint32_t max_value = (1u << n_bits) - 1u;   // n_bits is at most 24.

        if (value > max_value)
        {
            value = max_value;
            p_bits->b_error = true;
        }

        proto_put(p_bits, value, n_bits);
    }

    return;
}

void
proto_put_s (proto_bits_t * p_bits, int32_t value, uint8_t n_bits)
{
    if (false == b_width_ok(n_bits))
    {
        p_bits->b_error = true;
    }
    else
    {
        // NOTE: n_bits is 1 to 24, so half fits in int32_t.
        int32_t half = (int32_t) (1u << (n_bits - 1u));

        if (value < -half)
        {
            value = -half;
            p_bits->b_error = true;
        }
        else if (value > (half - 1))
        {
            value = half - 1;
            p_bits->b_error = true;
        }
        else
        {
            // Value is already inside the field's range.
        }

        // The sum is never negative here, so the cast does not change it.
        proto_put(p_bits, (uint32_t) (value + half), n_bits);
    }

    return;
}

uint32_t
proto_get (proto_bits_t * p_bits, uint8_t n_bits)
{
    uint32_t value = 0u;

    if (false == b_width_ok(n_bits))
    {
        p_bits->b_error = true;
    }
    else
    {
        for (uint8_t idx = 0u; idx < n_bits; idx++)
        {
            if (p_bits->pos_bits >= p_bits->cap_bits)
            {
                p_bits->b_error = true;
            }
            else
            {
                uint32_t byte_idx = p_bits->pos_bits / BITS_PER_BYTE;
                uint32_t shift    = p_bits->pos_bits % BITS_PER_BYTE;

                value |= (((uint32_t) p_bits->p_buf[byte_idx] >> shift) & 1u)
                         << idx;
                p_bits->pos_bits++;
            }
        }
    }

    return (value);
}

int32_t
proto_get_s (proto_bits_t * p_bits, uint8_t n_bits)
{
    int32_t result = 0;

    if (false == b_width_ok(n_bits))
    {
        p_bits->b_error = true;
    }
    else
    {
        // NOTE: n_bits is 1 to 24, so the casts to int32_t are safe.
        int32_t half = (int32_t) (1u << (n_bits - 1u));

        result = ((int32_t) proto_get(p_bits, n_bits)) - half;
    }

    return (result);
}

static bool
b_width_ok (uint8_t n_bits)
{
    return ((n_bits >= 1u) && (n_bits <= MAX_FIELD_BITS));
}

/*** end of file ***/