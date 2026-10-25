#include <tk/tkernel.h>
#include <sys/sysdef.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "wheel_encoder.h"

/* -----------------------------------------
 * 1. Encoder GPIO Configuration
 * ----------------------------------------- */

/* Left wheel encoder (M1) */
#define LEFT_ENCODER_A      16
#define LEFT_ENCODER_B      17

/* Right wheel encoder (M2) */
#define RIGHT_ENCODER_A     6
#define RIGHT_ENCODER_B     7


/* -----------------------------------------
 * 2. RP2040 GPIO Interrupt Configuration
 * ----------------------------------------- */

/* GPIO interrupt controller */
#define ENCODER_IO_BANK0_BASE   0x40014000u
#define ENCODER_GPIO_IRQ        13u
#define ENCODER_IRQ_PRIORITY    2

/* Raw GPIO interrupt status registers */
#define ENCODER_INTR0         (ENCODER_IO_BANK0_BASE + 0x0F0u)
#define ENCODER_INTR2         (ENCODER_IO_BANK0_BASE + 0x0F8u)

/* Core 0 GPIO interrupt enable registers */
#define ENCODER_PROC0_INTE0   (ENCODER_IO_BANK0_BASE + 0x100u)
#define ENCODER_PROC0_INTE2   (ENCODER_IO_BANK0_BASE + 0x108u)

/* Rising and falling edge detection */
#define ENCODER_EDGE_FALL  (1u << 2)
#define ENCODER_EDGE_RISE  (1u << 3)
#define ENCODER_EDGE_BOTH  (ENCODER_EDGE_FALL | ENCODER_EDGE_RISE)

/* Position of each GPIO's 4-bit interrupt field */
#define ENCODER_IRQ_SHIFT(pin)  (((pin) % 8u) * 4u)
#define ENCODER_IRQ_MASK(pin)  (ENCODER_EDGE_BOTH << ENCODER_IRQ_SHIFT(pin))

/* Interrupt masks for both encoders (M1 & M2) */
// M1 encoder -> GP16 and GP17 
#define LEFT_IRQ_MASK \
    (ENCODER_IRQ_MASK(LEFT_ENCODER_A) | ENCODER_IRQ_MASK(LEFT_ENCODER_B))

// M2 encoder -> GP6 and GP7
#define RIGHT_IRQ_MASK \
    (ENCODER_IRQ_MASK(RIGHT_ENCODER_A) | ENCODER_IRQ_MASK(RIGHT_ENCODER_B))

/* -----------------------------------------
 * 3. Encoder State Varaiables
 * ----------------------------------------- */

/* Updated by GPIO interrupts */
static volatile int32_t left_count = 0;
static volatile int32_t right_count = 0;

/* Previous A/B states used for quadrature decoding */
static uint8_t left_prev = 0;
static uint8_t right_prev = 0;

/* -----------------------------------------
 * 4. Quadrature Encoder Decoding
 * ----------------------------------------- */

/*
 * Lookup table for transitions between previous and current encoder A/B states.
 *
 * Valid transitions: +1 or -1.
 * No change / invalid transitions: 0
 */
static const int8_t encoder_table[16] = {
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0
};

/* Decode the transition between two encoder states */
static int8_t encoder_decode(uint8_t previous, uint8_t current)
{
    uint8_t index = (previous << 2) | current;
    return encoder_table[index];
}

/* Read both encoder A and B as a 2-bit state */
static uint8_t encoder_readState(int pin_a, int pin_b)
{
    uint32_t pins = in_w(GPIO_IN);

    uint8_t a = (pins >> pin_a) & 1u;
    uint8_t b = (pins >> pin_b) & 1u;

    return (a << 1) | b;
}


/* -----------------------------------------
 * 5. Encoder Initialisation
 * ----------------------------------------- */

static ER encoder_registerInterrupt(void);

void encoder_init(void)
{
    /* Set both encoder pins as GPIO input */
    gpio_set_pin(LEFT_ENCODER_A, GPIO_MODE_IN);
    gpio_set_pin(LEFT_ENCODER_B, GPIO_MODE_IN);
    gpio_set_pin(RIGHT_ENCODER_A, GPIO_MODE_IN);
    gpio_set_pin(RIGHT_ENCODER_B, GPIO_MODE_IN);

    /* Capture initial encoder states */
    left_prev = encoder_readState(LEFT_ENCODER_A, LEFT_ENCODER_B);
    right_prev = encoder_readState(RIGHT_ENCODER_A, RIGHT_ENCODER_B);

    /* Reset encoder counters */
    left_count = 0;
    right_count = 0;

    /* Register GPIO interrupt handler with micro T-Kernel */
    ER er = encoder_registerInterrupt();

    if (er != E_OK) {
        tm_printf((UB*)"[S2-ENC] Interrupt registration failed: %d\n", er);
    }

    if (er == E_OK) {
        /* Clear any pending encoder GPIO edge events */
        out_w(ENCODER_INTR0, RIGHT_IRQ_MASK);
        out_w(ENCODER_INTR2, LEFT_IRQ_MASK);
        
        /* Clear pending NVIC interrupt */
        ClearInt(ENCODER_GPIO_IRQ);

        /* Enable encoder edge interrupts on Core 0 */
        out_w(ENCODER_PROC0_INTE0,
            in_w(ENCODER_PROC0_INTE0) | RIGHT_IRQ_MASK);

        out_w(ENCODER_PROC0_INTE2,
            in_w(ENCODER_PROC0_INTE2) | LEFT_IRQ_MASK);

        /* Enable GPIO interrupt in Core 0 NVIC */
        EnableInt(ENCODER_GPIO_IRQ, ENCODER_IRQ_PRIORITY);
    }
}


/* -----------------------------------------
 * 6. Encoder Interrupt Handling
 * ----------------------------------------- */

/* Update encoder counts based on interrupt events only */
static void encoder_updateCounts(uint32_t left_events, uint32_t right_events)
{
    if (left_events != 0u) {
        uint8_t left_now =
            encoder_readState(LEFT_ENCODER_A, LEFT_ENCODER_B);

        /* Forward rotation increases the count */
        left_count -= encoder_decode(left_prev, left_now);
        left_prev = left_now;
    }

    if (right_events != 0u) {
        uint8_t right_now =
            encoder_readState(RIGHT_ENCODER_A, RIGHT_ENCODER_B);

        /* Forward rotation increases the count */
        right_count -= encoder_decode(right_prev, right_now);
        right_prev = right_now;
    }
}

/* GPIO interrupt handler shared by both encoders */
static void encoder_inthdr(void)
{
    /* Read pending encoder edge events */
    uint32_t left_events =
        in_w(ENCODER_INTR2) & LEFT_IRQ_MASK;

    uint32_t right_events =
        in_w(ENCODER_INTR0) & RIGHT_IRQ_MASK;

    /* Update counts before acknowledging the events */
    encoder_updateCounts(left_events, right_events);

    /* Clear the GPIO events handled */
    if (left_events != 0u) {
        out_w(ENCODER_INTR2, left_events);
    }

    if (right_events != 0u) {
        out_w(ENCODER_INTR0, right_events);
    }
}

/* Register GPIO IRQ 13 with micro T-Kernel */
static ER encoder_registerInterrupt(void)
{
    const T_DINT dint = {
        .intatr = TA_HLNG,
        .inthdr = (FP)encoder_inthdr,
    };

    return tk_def_int(ENCODER_GPIO_IRQ, &dint);
}


/* -----------------------------------------
 * 7. Encoder GPIO State Getters
 * ----------------------------------------- */

/* Read left encoder's current GPIO state */
int encoder_getLeftA(void)
{
    return gpio_get_val(LEFT_ENCODER_A);
}

int encoder_getLeftB(void)
{
    return gpio_get_val(LEFT_ENCODER_B);
}

/* Read right encoder's current GPIO state */
int encoder_getRightA(void)
{
    return gpio_get_val(RIGHT_ENCODER_A);
}

int encoder_getRightB(void)
{
    return gpio_get_val(RIGHT_ENCODER_B);
}


/* -----------------------------------------
 * 8. Encoder Count Getters
 * ----------------------------------------- */

/* Get the left wheel's encoder count */
int32_t encoder_getLeftCount(void)
{
    return left_count;
}

/* Get the right wheel's encoder count */
int32_t encoder_getRightCount(void)
{
    return right_count;
}