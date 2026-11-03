#ifndef DRV_SERVO_H
#define DRV_SERVO_H

#include <tk/tkernel.h>

#define SERVO_PIN           6   /* GP6: PWM Slice 3, Channel A */

#define SERVO_ANGLE_LEFT   -45
#define SERVO_ANGLE_CENTER   0
#define SERVO_ANGLE_RIGHT   45

void servo_init(void);
void servo_set_angle(INT angle_deg);

#endif /* DRV_SERVO_H */