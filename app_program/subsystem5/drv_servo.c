#include "drv_servo.h"
#include <bsp/libbsp.h>

/* RP2040 system clock is 133 MHz.
 * Using a clock divider of 133 gives a 1 MHz timer (1 tick = 1 us).
 * A wrap period of 20000 ticks gives 20 ms (50 Hz).
 */
void servo_init(void)
{
    gpio_set_pin(SERVO_PIN, GPIO_MODE_ALT); /* PWM function */
    pwm_init_pin(SERVO_PIN, 133.0f, 20000);
    servo_set_angle(SERVO_ANGLE_CENTER);
}

void servo_set_angle(INT angle_deg)
{
    /* Clamp angle between -45 and +45 */
    if (angle_deg < -45) angle_deg = -45;
    if (angle_deg > 45)  angle_deg = 45;

    /* Map -45 deg to 1000 us, 0 deg to 1500 us, +45 deg to 2000 us */
    UW pulse_width_us = 1500 + ((angle_deg * 500) / 45);
    pwm_set_duty_us(SERVO_PIN, pulse_width_us);
}