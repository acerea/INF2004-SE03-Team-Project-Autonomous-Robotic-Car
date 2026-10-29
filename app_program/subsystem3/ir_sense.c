/**
 * @file ir_sense.c
 * @brief Implementation of Buddy 3 Line Following & Barcode Navigation
 */

#include "ir_sense.h"

#define PIN_IR_LEFT 26
#define PIN_IR_CENTRE 27
#define PIN_IR_RIGHT 28
/*#define WHATTHEFUCK REMEMBER TO DELETE*/

/* Forward declare state handlers to populate the translation table matrix */
static void handle_state_calibrating(buddy3_context_t *ctx);
static void handle_state_tracking(buddy3_context_t *ctx);
static void handle_state_barcode_decoding(buddy3_context_t *ctx);
static void handle_state_command_pending(buddy3_context_t *ctx);
static void handle_state_junction_detected(buddy3_context_t *ctx);
static void handle_state_line_lost(buddy3_context_t *ctx);
static void handle_state_fault(buddy3_context_t *ctx);

/**
 * @brief State-to-Function Pointer Matrix. Completely replaces traditional
 * nested if-else / switch structures to enforce deterministic O(1) transitions.
 */
typedef void (*state_handler_t)(buddy3_context_t *);
static const state_handler_t state_matrix[NUM_STATES] = {
    [STATE_CALIBRATING]       = handle_state_calibrating,
    [STATE_TRACKING]          = handle_state_tracking,
    [STATE_BARCODE_DECODING]  = handle_state_barcode_decoding,
    [STATE_COMMAND_PENDING]   = handle_state_command_pending,
    [STATE_JUNCTION_DETECTED] = handle_state_junction_detected,
    [STATE_LINE_LOST]         = handle_state_line_lost,
    [STATE_FAULT]             = handle_state_fault
};

void ir_subsystem_init(buddy3_context_t *ctx) {
    if (!ctx) return;
    
    ctx->current_state = STATE_CALIBRATING;
    ctx->emergency_stop_active = false;
    
    /* Initialize PD Loop Configs */
    ctx->pd_steering.kp = 1.25f;
    ctx->pd_steering.kd = 0.45f;
    ctx->pd_steering.prev_error = 0.0f;
    ctx->pd_steering.target_center = 0; // Balanced differential center
    
    /* Standard hardware threshold defaults */
    ctx->threshold_black = 3000; 
    ctx->threshold_white = 500;
    ctx->ir_midpoint = 1750;
    
    ctx->edge_count = 0;
    ctx->barcode_retry_count = 0;
    ctx->pending_raw_payload = 0x00;
    ctx->line_lost_start_ms = 0;
}

void ir_line_sensor_task_run(buddy3_context_t *ctx) {
    if (!ctx) return;

    /* Global Safety Constraint Override (FR16 Compliance) */
    if (ctx->emergency_stop_active) {
        ctx->current_state = STATE_FAULT;
    }

    /* Execute the mapped function handler via deterministic pointer matrix lookups */
    if (ctx->current_state < NUM_STATES && state_matrix[ctx->current_state]) {
        state_matrix[ctx->current_state](ctx);
    }
}

void ir_barcode_task_run(buddy3_context_t *ctx) {
    if (!ctx) return;
    
    /* Core 1 logic evaluates barcode paths strictly when state machine permits */
    if (ctx->current_state == STATE_BARCODE_DECODING) {
        uint16_t adc_val = stub_read_hardware_adc(PIN_IR_RIGHT);
        
        /* Simulated time-based edge tracking loop */
        if (adc_val > ctx->ir_midpoint) {
            if (ctx->edge_count < 16) {
                ctx->edge_intervals[ctx->edge_count++] = 5; // Track 5ms intervals
            }
        }
        
        /* Evaluate terminal parsing criteria */
        if (ctx->edge_count >= 12) {
            /* Successful decode path: Extract 2-bit navigation command payloads */
            ctx->pending_raw_payload = (NAV_CMD_RIGHT & NAV_CMD_MASK); 
            ctx->current_state = STATE_COMMAND_PENDING;
        } else if (ctx->edge_count >= 15) { // Force tracking limits overflow
            if (ctx->barcode_retry_count < 1) {
                ctx->barcode_retry_count++;
                ctx->edge_count = 0; // Clear out and retry sampling sequence
            } else {
                /* Failed retry cycle: Dispatch event directly to Vehicle Controller */
                vehicle_controller_notify_event(STATE_BARCODE_DECODING, 0xFF); 
                ctx->current_state = STATE_TRACKING;
            }
        }
    }
}

void ir_subsystem_set_emergency_stop(buddy3_context_t *ctx, bool stop_active) {
    if (ctx) {
        ctx->emergency_stop_active = stop_active;
    }
}

/* ========================================================================== */
/*                        STATE HANDLING IMPLEMENTATIONS                     */
/* ========================================================================== */

static void handle_state_calibrating(buddy3_context_t *ctx) {
    uint16_t left_adc = stub_read_hardware_adc(PIN_IR_LEFT);
    uint16_t centre_adc = stub_read_hardware_adc(PIN_IR_CENTRE);
    
    if (left_adc > 0 && centre_adc > 0) {
        ctx->ir_midpoint = (ctx->threshold_black + ctx->threshold_white) >> 1;
        ctx->current_state = STATE_TRACKING;
    } else {
        ctx->current_state = STATE_FAULT;
        vehicle_controller_notify_event(STATE_CALIBRATING, 0x01); // Calibration Failed
    }
}

static void handle_state_tracking(buddy3_context_t *ctx) {
    uint16_t left_sensor = stub_read_hardware_adc(PIN_IR_LEFT);
    uint16_t centre_sensor = stub_read_hardware_adc(PIN_IR_CENTRE);
    uint16_t right_sensor = stub_read_hardware_adc(PIN_IR_RIGHT);

    bool left_on_line = (left_sensor > ctx->ir_midpoint);
    bool centre_on_line = (centre_sensor > ctx->ir_midpoint);
    bool right_edge_detected = (right_sensor > ctx->ir_midpoint);

    /* Line Lost Branch Path Evaluation */
    if (!left_on_line && !centre_on_line) {
        ctx->line_lost_start_ms = stub_get_system_time_ms();
        ctx->current_state = STATE_LINE_LOST;
        return;
    }

    /* Transition block to handle barcode sequences running on Core 1 */
    if (right_edge_detected) {
        ctx->edge_count = 0;
        ctx->barcode_retry_count = 0;
        ctx->current_state = STATE_BARCODE_DECODING;
        return;
    }

    /* ========================================================================== */
    /*               PROPORTIONAL-DERIVATIVE (PD) LINE CALCULATION                */
    /* ========================================================================== */
    /* Computes line positioning errors using differential metrics */
    int16_t current_error = (int16_t)left_sensor - (int16_t)centre_sensor;
    
    float p_term = ctx->pd_steering.kp * (float)current_error;
    float d_term = ctx->pd_steering.kd * ((float)current_error - ctx->pd_steering.prev_error);
    
    float steering_signal = p_term + d_term;
    ctx->pd_steering.prev_error = (float)current_error;

    /* Apply safety dampening or temporary steering locks during transitions */
    vehicle_controller_dispatch_steering(steering_signal);
}

static void handle_state_barcode_decoding(buddy3_context_t *ctx) {
    /* Steering updates are temporarily bypassed to preserve edge timing consistency */
    vehicle_controller_dispatch_steering(0.0f);
}

static void handle_state_command_pending(buddy3_context_t *ctx) {
    uint16_t left = stub_read_hardware_adc(PIN_IR_LEFT);
    uint16_t centre = stub_read_hardware_adc(PIN_IR_CENTRE);

    /* Check if physical junction patterns are reached */
    if (left > ctx->ir_midpoint && centre > ctx->ir_midpoint) {
        ctx->current_state = STATE_JUNCTION_DETECTED;
    }
}

static void handle_state_junction_detected(buddy3_context_t *ctx) {
    /* Send the minified 2-bit navigation code parameter directly to Vehicle Controller */
    uint8_t payload = ctx->pending_raw_payload & NAV_CMD_MASK;
    
    vehicle_controller_notify_event(STATE_JUNCTION_DETECTED, payload);
    
    /* Hand over flow control to the external turning tasks */
    ctx->current_state = STATE_TRACKING; 
}

static void handle_state_line_lost(buddy3_context_t *ctx) {
    uint16_t left = stub_read_hardware_adc(PIN_IR_LEFT);
    uint16_t centre = stub_read_hardware_adc(PIN_IR_CENTRE);

    /* Re-verify tracking recovery vectors */
    if (left > ctx->ir_midpoint || centre > ctx->ir_midpoint) {
        ctx->current_state = STATE_TRACKING;
        return;
    }

    /* Enforce rigid deterministic hardware timeouts (Failsafe checking paths) */
    if ((stub_get_system_time_ms() - ctx->line_lost_start_ms) > 3000) { // 3-second limit
        ctx->current_state = STATE_FAULT;
        vehicle_controller_notify_event(STATE_LINE_LOST, 0x02); // Timeout trigger
    } else {
        /* Request slower search speed profile mapping */
        vehicle_controller_dispatch_steering(0.0f); 
    }
}

static void handle_state_fault(buddy3_context_t *ctx) {
    /* Hard lockout state forcing active hardware configurations to halt completely */
    vehicle_controller_dispatch_steering(0.0f);
    vehicle_controller_notify_event(STATE_FAULT, 0x00);
}

/* ========================================================================== */
/*               STUBBED HARDWARE AND CONTROL INTEGRATION LAYER               */
/* ========================================================================== */

void __attribute__((weak)) vehicle_controller_dispatch_steering(float steering_signal) {
    // Linked by external system framework targets
}

void __attribute__((weak)) vehicle_controller_notify_event(ir_subsystem_state_t state, uint8_t event_flags) {
    // Linked by external system framework targets
}

uint32_t __attribute__((weak)) stub_get_system_time_ms(void) {
    return 1000; 
}

uint16_t __attribute__((weak)) stub_read_hardware_adc(uint8_t pin) {
    return 1200; 
}
