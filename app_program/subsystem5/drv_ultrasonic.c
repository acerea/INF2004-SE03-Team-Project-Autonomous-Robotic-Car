#include "drv_ultrasonic.h"
#include <tstdlib.h>
#include <tk/tkernel.h>
#include <bsp/libbsp.h>

/* Pin configuration for Subsystem 5 Ultrasonic Sensor */
#define TRIG_PIN       2
#define ECHO_PIN       3
#define TIMEOUT_US     35000   /* 35 ms timeout */

/* RP2040 Hardware Timer Raw Lower 32-bit register (1 count = 1 us) */
#define RP2040_TIMER_RAWL      (*(volatile UW *)(0x40054000 + 0x28))

/* RP2040 Low-Level Control Registers */
#define IO_BANK0_GPIO_CTRL(n)  (*(volatile UW *)(0x40014000 + 0x04 + ((n) * 8)))
#define PADS_BANK0_GPIO(n)     (*(volatile UW *)(0x4001c000 + 0x04 + ((n) * 4)))

static inline UW get_time_us(void)
{
    return RP2040_TIMER_RAWL;
}

void ultrasonic_init(void)
{
    /* 1. Connect GP2 and GP3 to the GPIO controller (FUNCSEL = 5) */
    IO_BANK0_GPIO_CTRL(TRIG_PIN) = 5;
    IO_BANK0_GPIO_CTRL(ECHO_PIN) = 5;

    /* 2. Set pin directions */
    gpio_set_pin(TRIG_PIN, GPIO_MODE_OUT);
    gpio_set_val(TRIG_PIN, 0);

    gpio_set_pin(ECHO_PIN, GPIO_MODE_IN);

    /* 3. Turn on the input buffer on GP3 (bit 6 = IE) */
    PADS_BANK0_GPIO(ECHO_PIN) = (1 << 6);

    /* 4. Disable pad isolation on both pins (bit 8 = ISO) */
    PADS_BANK0_GPIO(TRIG_PIN) &= ~(1 << 8);
    PADS_BANK0_GPIO(ECHO_PIN) &= ~(1 << 8);
}

float ultrasonic_read_distance_cm(void)
{
    /* 1. Settle LOW for 10 us */
    gpio_set_val(TRIG_PIN, 0);
    UW t0 = get_time_us();
    while ((get_time_us() - t0) < 10);

    /* 2. Emit 10 us trigger pulse */
    gpio_set_val(TRIG_PIN, 1);
    UW t1 = get_time_us();
    while ((get_time_us() - t1) < 12);
    gpio_set_val(TRIG_PIN, 0);

    /* 3. Wait for Echo to go HIGH (give up after 15 ms) */
    UW t_start = get_time_us();
    while (gpio_get_val(ECHO_PIN) == 0) {
        if ((get_time_us() - t_start) > 15000) {
            return -1.0f;
        }
    }

    /* 4. Measure duration while Echo is HIGH (give up after 35 ms) */
    UW t_echo_start = get_time_us();
    while (gpio_get_val(ECHO_PIN) != 0) {
        if ((get_time_us() - t_echo_start) > TIMEOUT_US) {
            return -1.0f;
        }
    }
    UW duration_us = get_time_us() - t_echo_start;

    /* Convert flight time to distance in cm */
    return ((float)duration_us * 0.0343f) / 2.0f;
}

/* Helper: Sort 3 floats and return median */
static float median_of_3(float a, float b, float c)
{
    if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
    if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
    return c;
}

float ultrasonic_read_distance_filtered(void)
{
    float r1 = ultrasonic_read_distance_cm();
    tk_dly_tsk(15); /* Short acoustic decay */

    float r2 = ultrasonic_read_distance_cm();
    tk_dly_tsk(15);

    float r3 = ultrasonic_read_distance_cm();

    /* If all 3 timed out, return -1.0f (clean open air) */
    if (r1 < 0.0f && r2 < 0.0f && r3 < 0.0f) {
        return -1.0f;
    }

    /* If 1 sample timed out but other 2 are valid, use the valid average */
    if (r1 < 0.0f && r2 > 0.0f && r3 > 0.0f) return (r2 + r3) / 2.0f;
    if (r2 < 0.0f && r1 > 0.0f && r3 > 0.0f) return (r1 + r3) / 2.0f;
    if (r3 < 0.0f && r1 > 0.0f && r2 > 0.0f) return (r1 + r2) / 2.0f;

    return median_of_3(r1, r2, r3);
}