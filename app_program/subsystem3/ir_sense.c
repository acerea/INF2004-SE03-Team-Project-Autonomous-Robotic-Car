#include <stdint.h>
#include "ir_sense.h"
#include <tk/tkernel.h>
#include <string.h>

#define PIN_IR_LEFT 26
#define PIN_IR_CENTRE 27
#define PIN_IR_RIGHT 28

//adc channel mapping
#define ADC_LEFT   0   /* GPIO26 = Channel 0 */
#define ADC_CENTRE 1   /* GPIO27 = Channel 1 */
#define ADC_RIGHT  2   /* GPIO28 = Channel 2 */

#define bthreshold 2000 //threshold of which the ir sensor reads blacklines
#define wthreshold 500 //threshold of which the ir sensor reads white area
#define TIMEOUTLOSS 1000 //timout boundary for execute_line_lost(void)

#define MAXPULSE 9 //9 elements for barcode character
#define BLANKS_MS 130 //white space/blanks duration

static buddy3_context_t context;
static ID ir_mutex_id;
static volatile bool steering_locked = false;

//read raw analag data from gpio/adc channel
static inline uint16_t read_adc_raw(uint8_t channel) {
    adc_select_input(channel);
    return adc_read();
}

typedef void (*state_handler_t)(void);

//sensor calibration 
static void execute_cali(void) {
    tk_loc_mtx(ir_mutex_id, TMO_FEVR);
    context.threshold_black = bthreshold;
    context.threshold_white = wthreshold;
    context.current_state.current_state = IR_TRACK;
    tk_unl_mtx(ir_mutex_id);
}

//line following and estimation
static void execute_track(void) {

    context.current_state.current_state = IR_TRACK & IR_MASK;

    if (steering_locked) { //no line correction if steering locked
        return;
    }

    //read adc values
    uint16_t left_val = read_adc_raw(ADC_LEFT);
    uint16_t centre_val = read_adc_raw(ADC_CENTRE);
    uint16_t right_val = read_adc_raw(ADC_RIGHT);

    tk_loc_mtx(ir_mutex_id, TMO_FEVR);

    //if side sensors are safe on white, but centre catches black
    if (left_val < wthreshold && right_val < wthreshold && centre_val >= bthreshold) {
        context.current_state.current_state = IR_BARCODE;
        context.barcode_started = false;
        context.pulse_count = 0;
        context.string_index = 0;
        context.decoded_string[0] = '\0';
        context.last_transition_time = tk_ref_slt();
        context.last_was_black = true; // Initialized to true because center is black
        tk_unl_mtx(ir_mutex_id);
        return;
    }

    //traditional line-loss fallback logic
    if (left_val < wthreshold && centre_val < wthreshold) { //might be bthresh need to test further but if sensor data is under white threshold the line is lost
        context.current_state.current_state = IR_LINE_LOST;
        context.line_lost_start_ms = tk_ref_slt();
    }
    
    tk_unl_mtx(ir_mutex_id);

}

static char lookup_barcode_char(uint16_t bit_pattern) {
    for (size_t i = 0; i < LOOKUP_SIZE; i++) {
        if (nav_letters[i].pattern == bit_pattern) {
            return nav_letters[i].alphabet; // return matching alpha/ASCII character
        }
    }
    return '?'; // error - placeholder if signature is corrupted
}

// Convert the 9 accumulated pulse timings into an ASCII character
static char process_accumulated_pulses(uint16_t *widths, uint8_t count) {
    if (count < 9) return '\0';

    //Calculate dynamic average width threshold
    uint32_t total = 0;
    for (int i = 0; i < 9; i++) {
        total += widths[i];
    }
    uint32_t average_width = total / 9;

    //Map widths into a 9-bit binary string representation
    uint16_t bit_pattern = 0;
    for (int i = 0; i < 9; i++) {
        if (widths[i] > average_width) {
            bit_pattern |= (1 << (8 - i)); // Element classified as Wide (1)
        }
    }

    //Scan the dictionary table structure
    return lookup_barcode_char(bit_pattern);
}

//read barcode and generate navigation commands
// Non-blocking barcode pulse width accumulator and string parser
static void execute_barcode(void) {
    context.current_state.current_state = IR_BARCODE & IR_MASK;

    uint16_t centre_val = read_adc_raw(ADC_CENTRE);
    uint32_t current_time = tk_ref_slt();
    uint32_t duration = current_time - context.last_transition_time;
    bool is_black = (centre_val >= bthreshold);

    tk_loc_mtx(ir_mutex_id, TMO_FEVR);

    //dynamic transitions capture
    if (is_black != context.last_was_black) {
        if (context.pulse_count < MAXPULSE) {
            context.pulse_widths[context.pulse_count++] = (uint16_t)duration;
        }
        
        context.last_transition_time = current_time;
        context.last_was_black = is_black;
        context.barcode_started = true;

        // letter sequence processing
        if (context.pulse_count == MAXPULSE) {
            char parsed_char = process_accumulated_pulses(context.pulse_widths, context.pulse_count);
            
            // skip bad captures
            if (parsed_char != '?' && parsed_char != '*' && parsed_char != '\0') {
                if (context.string_index < (sizeof(context.decoded_string) - 1)) {
                    context.decoded_string[context.string_index++] = parsed_char;
                    context.decoded_string[context.string_index] = '\0'; // Maintain safe null termination
                }
            }
            context.pulse_count = 0; //reset pulse
        }
    } 
    // end-of-barcode handling via white space quiet zone detection
    else if (!is_black && context.barcode_started && (duration > BLANKS_MS)) {
        
        // execute operational rules based on completed keywords parsed into string array
        if (strcmp(context.decoded_string, "STOP") == 0) {
            context.emergency_stop_active = true;
        } else if (strcmp(context.decoded_string, "LEFT") == 0) {
            context.current_state.junction_detected = 1; 
        }

        // reset tracking configurations cleanly
        context.barcode_started = false;
        context.pulse_count = 0;
        context.string_index = 0;
        context.decoded_string[0] = '\0';
        
        // transition to navigation routing
        context.current_state.current_state = IR_TRACK;
    }
    // fallback tracking protection if stuck on an unexpected solid bar line mid-scan
    else if (is_black && (duration > TIMEOUTLOSS)) {
        context.current_state.current_state = IR_LINE_LOST;
        context.line_lost_start_ms = current_time;
    }

    tk_unl_mtx(ir_mutex_id);
}
//detect line loss and response
static void execute_line_lost(void) {

    context.current_state.current_state = IR_LINE_LOST & IR_MASK;

    //initiate timer for first line loss instance
    if (context.line_lost_start_ms == 0) {
        context.line_lost_start_ms = tk_ref_slt();
    }
    
    //timeout boundary if exceeded emergency stop
    if ((tk_ref_slt() - context.line_lost_start_ms) > TIMEOUTLOSS) {
        context.emergency_stop_active = true;
    }

}

//function table mapping to logical callback
static const state_handler_t state_execution_table[] = {
    [IR_CALI]      = execute_cali,
    [IR_TRACK]     = execute_track,
    [IR_BARCODE]   = execute_barcode,
    [IR_LINE_LOST] = execute_line_lost
};

void ir_get_telemetry(ir_subsystem_state_t *out_snapshot) {

    tk_loc_mtx(ir_mutex_id, TMO_FEVR);

    out_snapshot->timestamp          = context.current_state.timestamp;
    out_snapshot->estimated_line_pos = context.current_state.estimated_line_pos;
    out_snapshot->junction_detected  = context.current_state.junction_detected;
    out_snapshot->current_state      = context.current_state.current_state & IR_MASK;

    tk_unl_mtx(ir_mutex_id);
}


//toglle for steering lock
void ir_lock_steering(bool lock) {

    steering_locked = lock;
}


//ir sensor task core 0
static void ir_sensor_task(INT stacd, void *exinf) {

    while (1) {

        uint8_t target_state = context.current_state.current_state & IR_MASK;
        state_execution_table[target_state]();
        context.current_state.timestamp = tk_ref_slt();
        tk_dly_tsk(10);
    }
}


//configurations
void ir_sense_init(void) {

    adc_init();
    adc_gpio_init(PIN_IR_LEFT);
    adc_gpio_init(PIN_IR_CENTRE);
    adc_gpio_init(PIN_IR_RIGHT);

    //fifo RTOS Mutex for clean thread boundary sync
    T_CMTX cmtx = {
        .mtxatr = TA_TFIFO | TA_INHERIT
    };

    ir_mutex_id = tk_cre_mtx(&cmtx);
    //default startup
    context.current_state.current_state = IR_CALI;
    context.current_state.estimated_line_pos = 0;
    context.current_state.junction_detected = 0;
    context.line_lost_start_ms = 0;
    context.emergency_stop_active = false;
    context.edge_count = 0;
    context.barcode_retry_count = 0;

    context.barcode_started = false;
    context.pulse_count = 0;
    context.string_index = 0;
    context.decoded_string[0] = '\0';
}   

//starts execution
void ir_sense_start(void) {

    T_CTSK ctsk = {
        .tskatr  = TA_HLNG | TA_RNG0,
        .task    = ir_sensor_task,
        .itskpri = 3,  //high priority
        .stksz   = 1048
    };

    ID task_id = tk_cre_tsk(&ctsk);
    tk_sta_tsk(task_id, 0);
}