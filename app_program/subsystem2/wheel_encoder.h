#ifndef WHEEL_ENCODER_H
#define WHEEL_ENCODER_H

#include <stdint.h>

/* Initialise both wheel encoders */
void encoder_init(void);

/* Get encoder counts */
int32_t encoder_getLeftCount(void);
int32_t encoder_getRightCount(void);

/* Read encoder GPIO states */
int encoder_getLeftA(void);
int encoder_getLeftB(void);
int encoder_getRightA(void);
int encoder_getRightB(void);

/* Get wheel distance (cm) */
float encoder_getLeftDistance(void);
float encoder_getRightDistance(void);

/* Calculate average wheel speed (cm/s) */
float encoder_calculateLeftSpeed(int32_t count_change, float dt_seconds);
float encoder_calculateRightSpeed(int32_t count_change, float dt_seconds);

#endif /* WHEEL_ENCODER_H */