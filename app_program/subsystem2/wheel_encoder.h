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

#endif /* WHEEL_ENCODER_H */