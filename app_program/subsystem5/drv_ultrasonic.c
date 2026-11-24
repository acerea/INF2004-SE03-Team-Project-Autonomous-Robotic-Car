#include "drv_ultrasonic.h"
#include <tstdlib.h>
#include <tk/tkernel.h>
#include <bsp/libbsp.h>

/* Pin configuration for Subsystem 5 Ultrasonic Sensor */
#define TRIG_PIN       2
#define ECHO_PIN       3
/* Busy-wait limits. The task spins while it times the echo, so the waits
 * are kept short: the old 15 ms + 35 ms (x3 pings per reading) starved
 * lower-priority tasks on that core.
 *   RISE_TIMEOUT_US  the HC-SR04 raises ECHO 0.5-2 ms after TRIG; clones vary
 *   ECHO_MAX_US      12 ms = about 205 cm; anything further is reported
 *                    as no echo (-1), which only ever means "clear"
 *   POLL_GAP_US      two polls further apart than this mean the task was
 *                    preempted or interrupted while timing: the edge time
 *                    is unreliable (reads too far), so ping again */
#define RISE_TIMEOUT_US  15000u  /* as originally: some clones answer late */
#define ECHO_MAX_US      12000u
#define POLL_GAP_US      150u
#define PING_TRIES       3
#define ECHO_IDLE_MS     40      /* longest a stale echo can stay high */

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
    PADS_BANK0_GPIO(ECHO_PIN) = (1 << 6) | (1 << 2);   /* IE + pull-down */

    /* 4. Disable pad isolation on both pins (bit 8 = ISO) */
    PADS_BANK0_GPIO(TRIG_PIN) &= ~(1 << 8);
    PADS_BANK0_GPIO(ECHO_PIN) &= ~(1 << 8);
}

/* One ping. *p_disturbed is set if the timing loop was interrupted. */
static float ultrasonic_ping(BOOL *p_disturbed)
{
    UW last;
    UW now;

    *p_disturbed = FALSE;

    /* 0. A previous ping that we stopped timing (> ECHO_MAX_US) may still
     *    hold ECHO high for up to ~38 ms: wait it out, sleeping. */
    for (INT i = 0; (i < ECHO_IDLE_MS / 2) && (gpio_get_val(ECHO_PIN) != 0); i++) {
        tk_dly_tsk(2);
    }

    /* 1. Settle LOW for 10 us */
    gpio_set_val(TRIG_PIN, 0);
    UW t0 = get_time_us();
    while ((get_time_us() - t0) < 10);

    /* 2. Emit 10 us trigger pulse */
    gpio_set_val(TRIG_PIN, 1);
    UW t1 = get_time_us();
    while ((get_time_us() - t1) < 12);
    gpio_set_val(TRIG_PIN, 0);

    /* 3. Wait for Echo to go HIGH */
    UW t_start = get_time_us();
    last = t_start;
    while (gpio_get_val(ECHO_PIN) == 0) {
        now = get_time_us();
        if ((now - t_start) > RISE_TIMEOUT_US) {
            return -2.0f;   /* No answer at all: sensor disconnected/unpowered */
        }
        if ((now - last) > POLL_GAP_US) {
            *p_disturbed = TRUE;
        }
        last = now;
    }

    /* 4. Measure duration while Echo is HIGH */
    UW t_echo_start = get_time_us();
    last = t_echo_start;
    while (gpio_get_val(ECHO_PIN) != 0) {
        now = get_time_us();
        if ((now - t_echo_start) > ECHO_MAX_US) {
            return -1.0f;   /* Nothing within ~2 m */
        }
        if ((now - last) > POLL_GAP_US) {
            *p_disturbed = TRUE;
        }
        last = now;
    }
    UW duration_us = get_time_us() - t_echo_start;

    /* Convert flight time to distance in cm */
    return ((float)duration_us * 0.0343f) / 2.0f;
}

float ultrasonic_read_distance_cm(void)
{
    BOOL  b_disturbed = FALSE;
    float d = -1.0f;

    /* Ping again if the timing was interrupted (up to PING_TRIES); the
     * last attempt is used as it is. */
    for (INT i = 0; i < PING_TRIES; i++) {
        d = ultrasonic_ping(&b_disturbed);
        if (!b_disturbed) {
            break;
        }
        tk_dly_tsk(10);     /* let the old echo die away */
    }

    return d;
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

    /* If all 3 failed: -2 if the sensor never answered, else -1 (open air) */
    if (r1 < 0.0f && r2 < 0.0f && r3 < 0.0f) {
        return (r1 < -1.5f && r2 < -1.5f && r3 < -1.5f) ? -2.0f : -1.0f;
    }

    /* Only one valid echo: keep it, so two dropouts cannot hide an obstacle */
    if (r1 > 0.0f && r2 < 0.0f && r3 < 0.0f) return r1;
    if (r2 > 0.0f && r1 < 0.0f && r3 < 0.0f) return r2;
    if (r3 > 0.0f && r1 < 0.0f && r2 < 0.0f) return r3;

    /* If 1 sample timed out but other 2 are valid, use the valid average */
    if (r1 < 0.0f && r2 > 0.0f && r3 > 0.0f) return (r2 + r3) / 2.0f;
    if (r2 < 0.0f && r1 > 0.0f && r3 > 0.0f) return (r1 + r3) / 2.0f;
    if (r3 < 0.0f && r1 > 0.0f && r2 > 0.0f) return (r1 + r2) / 2.0f;

    return median_of_3(r1, r2, r3);
}