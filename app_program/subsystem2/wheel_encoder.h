#ifndef WHEEL_ENCODER_H
#define WHEEL_ENCODER_H

#include <stdint.h>

/* Initialise both wheel encoders */
void encoder_init(void);

/* Get encoder counts */
int32_t encoder_get_left_count(void);
int32_t encoder_get_right_count(void);

/* Read encoder GPIO states */
int encoder_get_left_a(void);
int encoder_get_left_b(void);
int encoder_get_right_a(void);
int encoder_get_right_b(void);

/* Get wheel distance (cm) */
float encoder_get_left_distance(void);
float encoder_get_right_distance(void);

/* Calculate average wheel speed (cm/s) */
float encoder_calculate_left_speed(int32_t count_change, float dt_seconds);
float encoder_calculate_right_speed(int32_t count_change, float dt_seconds);

#endif /* WHEEL_ENCODER_H */