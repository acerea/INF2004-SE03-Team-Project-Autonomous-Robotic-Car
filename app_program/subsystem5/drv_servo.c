#include "drv_servo.h"
#include <bsp/libbsp.h>
#include <sys/machine.h>

/* RP2040 PWM hardware register defines matching libbsp_pwm.c */
// #define PWM_BASE            0x40050000
// #define PWM_CHx_DIV         0x04
#define PWM_CH_DIV(n)       (PWM_BASE + PWM_CHx_DIV + ((n) * 0x14))
#define PWM_GET_CHNO(pin)   (((pin) >> 1) & 0x07)

void servo_init(void)
{
    UINT ch = PWM_GET_CHNO(SERVO_PIN);

    /* 1. Set clock divider to 125 (gives 1 MHz timer -> 1 tick = 1 us)
     * Register format: [11:4] integer part, [3:0] fractional part */
    out_w(PWM_CH_DIV(ch), (125u << 4));

    /* 2. Set wrap for 20 ms period (50 Hz): 20000 us */
    pwm_set_wrap(SERVO_PIN, 19999);

    /* 3. Configure pin function to PWM */
    pwm_set_pin(SERVO_PIN);

    /* 4. Center the servo (1500 us) and enable PWM output */
    servo_set_angle(SERVO_ANGLE_CENTER);
    pwm_set_enabled(SERVO_PIN, TRUE);
}

void servo_set_angle(INT angle_deg)
{
    /* Clamp angle between -45 and +45 */
    if (angle_deg < -45) angle_deg = -45;
    if (angle_deg > 45)  angle_deg = 45;

    /* Linearly map:
     * -45 deg -> 1000 us
     *   0 deg -> 1500 us
     * +45 deg -> 2000 us */
    UW pulse_width_us = (UW)(1500 + ((angle_deg * 500) / 45));

    pwm_set_cc(SERVO_PIN, pulse_width_us);
}