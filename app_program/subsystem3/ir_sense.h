/**
 * @file ir_sense.h
 * @brief Buddy 3 Subsystem - Line Following & Barcode Navigation Interface
 * 
 * Target: Raspberry Pi Pico (RP2040) / Robo Pico Board
 * Implements a dual-core safe architecture:
 * - Core 0: Real-time PD Line-Following Control (10ms periodic loop)
 * - Core 1: Barcode Edge-Interval Parsing (5ms periodic loop)
 * 
 * Features state-to-function pointer translation matrices and minimum-bit data 
 * representations to eliminate multi-core message passing overhead.
 */

#ifndef IR_SENSE_H
#define IR_SENSE_H

#include <stdint.h>
#include <stdbool.h>

/* ========================================================================== */
/*                           HARDWARE PIN MAPPINGS                            */
/* ========================================================================== */
#define PIN_IR_LEFT         26  /**< ADC0/GP26: Left IR Line Sensor Input     */
#define PIN_IR_CENTRE       27  /**< ADC1/GP27: Centre IR Line Sensor Input   */
#define PIN_IR_RIGHT        28  /**< ADC2/GP28: Right IR Barcode Sensor Input */

/* ========================================================================== */
/*                        MINIMUM-BIT DATA PACKING                           */
/* ========================================================================== */
/**
 * @brief Navigation Commands compressed to a 2-bit footprint to eliminate
 * inter-core string serialization delays.
 */
typedef enum {
    NAV_CMD_LEFT     = 0x00,    /**< 00: Execute left turn at next junction      */
    NAV_CMD_RIGHT    = 0x01,    /**< 01: Execute right turn at next junction     */
    NAV_CMD_STRAIGHT = 0x02,    /**< 10: Execute straight movement at junction  */
    NAV_CMD_UTURN    = 0x03     /**< 11: Execute U-turn maneuver at junction    */
} nav_cmd_t;

#define NAV_CMD_MASK       0x03 /**< Bitmask to isolate 2-bit command data        */
#define JUNCTION_NO_CODE   0x04 /**< Flag signaling a junction with no barcode   */

/* ========================================================================== */
/*                       SUBSYSTEM STATE DEFINITIONS                          */
/* ========================================================================== */
typedef enum {
    STATE_CALIBRATING = 0,
    STATE_TRACKING,
    STATE_BARCODE_DECODING,
    STATE_COMMAND_PENDING,
    STATE_JUNCTION_DETECTED,
    STATE_LINE_LOST,
    STATE_FAULT,
    NUM_STATES
} ir_subsystem_state_t;

/* ========================================================================== */
/*                      PD CONTROLLER CONFIGURATION                           */
/* ========================================================================== */
typedef struct {
    float kp;                   /**< Proportional Gain                        */
    float kd;                   /**< Derivative Gain                          */
    float prev_error;           /**< Error memory storage for derivative step */
    int16_t target_center;      /**< Balanced line tracking reference point    */
} pd_config_t;

/* ========================================================================== */
/*                        SUBSYSTEM CONTEXT BLOCKS                            */
/* ========================================================================== */
typedef struct {
    ir_subsystem_state_t current_state;
    pd_config_t pd_steering;
    
    /* Sensor Thresholding Context */
    uint16_t threshold_black;
    uint16_t threshold_white;
    uint16_t ir_midpoint;
    
    /* Barcode Decoder Status */
    uint32_t edge_intervals[16];
    uint8_t edge_count;
    uint8_t barcode_retry_count;
    uint8_t pending_raw_payload; /**< Bit-packed 2-bit navigation payload    */
    
    /* Line Lost Safety Monitoring */
    uint32_t line_lost_start_ms;
    bool emergency_stop_active;
} buddy3_context_t;

/* ========================================================================== */
/*                         EXPOSED SYSTEM API                                 */
/* ========================================================================== */

/**
 * @brief Performs subsystem context bindings and configures physical pins.
 */
void ir_subsystem_init(buddy3_context_t *ctx);

/**
 * @brief Core 0 Execution Entry-point. Dispatched inside a 10ms RTOS task.
 * Executes PD tracking computations and handles state machine translations.
 */
void ir_line_sensor_task_run(buddy3_context_t *ctx);

/**
 * @brief Core 1 Execution Entry-point. Dispatched inside a 5ms RTOS task.
 * Performs precision edge tracking and non-blocking interval extraction.
 */
void ir_barcode_task_run(buddy3_context_t *ctx);

/**
 * @brief Event entry point allowing the Vehicle Controller to apply E-Stops.
 */
void ir_subsystem_set_emergency_stop(buddy3_context_t *ctx, bool stop_active);

/* ========================================================================== */
/*                 VEHICLE CONTROLLER & STUB INTERFACES                      */
/* ========================================================================== */
void vehicle_controller_dispatch_steering(float steering_signal);
void vehicle_controller_notify_event(ir_subsystem_state_t state, uint8_t event_flags);
uint32_t stub_get_system_time_ms(void);
uint16_t stub_read_hardware_adc(uint8_t pin);

#endif /* IR_SENSE_H */
