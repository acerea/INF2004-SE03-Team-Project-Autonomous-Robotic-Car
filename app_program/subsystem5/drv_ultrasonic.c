#include "drv_ultrasonic.h"
#include <tstdlib.h>
#include <tk/tkernel.h>
#include <bsp/libbsp.h>

/* Pin configuration for Subsystem 5 Ultrasonic Sensor */
#define TRIG_PIN       2
#define ECHO_PIN       3
#define TIMEOUT_US     30000   /* 30 ms maximum timeout */

/* Calibrated microsecond busy-wait loop for RP2040 (~133 MHz) */
static inline void delay_us(UW us)
{
    for (volatile UW i = 0; i < (us * 22); i++) {
        __asm__ volatile("nop");
    }
}

void ultrasonic_init(void)
{
    /* Configure TRIG_PIN as output and set initial state to LOW */
    gpio_set_pin(TRIG_PIN, GPIO_MODE_OUT);
    gpio_set_val(TRIG_PIN, 0);

    /* Configure ECHO_PIN as input */
    gpio_set_pin(ECHO_PIN, GPIO_MODE_IN);
}

float ultrasonic_read_distance_cm(void)
{
    /* 1. Transmit 10 us trigger pulse */
    gpio_set_val(TRIG_PIN, 1);
    delay_us(10);
    gpio_set_val(TRIG_PIN, 0);

    /* 2. Wait for Echo pin to rise HIGH (with timeout) */
    UW elapsed_wait = 0;
    while (gpio_get_val(ECHO_PIN) == 0) {
        delay_us(1);
        elapsed_wait++;
        if (elapsed_wait > TIMEOUT_US) {
            return -1.0f;
        }
    }

    /* 3. Measure duration while Echo pin remains HIGH */
    UW duration_us = 0;
    while (gpio_get_val(ECHO_PIN) != 0) {
        delay_us(1);
        duration_us++;
        if (duration_us > TIMEOUT_US) {
            return -1.0f;
        }
    }

    /* Distance = (Time * Speed of Sound 343 m/s) / 2 */
    return (float)duration_us * 0.0343f / 2.0f;
}
/* Standard memory operations needed when linking without libc */
void *memcpy(void *dest, const void *src, unsigned int n)
{
    char *d = (char *)dest;
    const char *s = (const char *)src;
    while (n--) {
        *d++ = *s++;
    }
    return dest;
}

void *memset(void *s, int c, unsigned int n)
{
    unsigned char *p = (unsigned char *)s;
    while (n--) {
        *p++ = (unsigned char)c;
    }
    return s;
}