#include "ir_sense.h"
#include "hardware/adc.h"
#include <tk/tkernel.h>
#include "hardware/gpio.h"

#define PIN_IR_LEFT 26
#define PIN_IR_CENTRE 27
#define PIN_IR_RIGHT 28
/*#define WHATTHEFUCK REMEMBER TO DELETE*/

#define bthreshold 2000 //threshold of which the ir sensor reads blacklines
#define wthreshold 500 //threshold of which the ir sensor reads white area
#define TIMEOUTLOSS 1000 //timout boundary for execute_line_lost(void)
/*my head hurts too much to exist*/

//adc channel mapping
#define ADC_LEFT   0   /* GPIO26 = Channel 0 */
#define ADC_CENTRE 1   /* GPIO27 = Channel 1 */
#define ADC_RIGHT  2   /* GPIO28 = Channel 2 */

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
    context.threshold_white =wthreshold;
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

    tk_loc_mtx(ir_mutex_id, TMO_FEVR);
    if (left_val < wthreshold && centre_val < wthreshold) { //might be bthresh need to test further but if sensor data is under white threshold the line is lost
        context.current_state.current_state = IR_LINE_LOST;
        context.line_lost_start_ms = tk_ref_slt();
    }
    tk_unl_mtx(ir_mutex_id);

}

//read barcode and generate navigation commands
static void execute_barcode(void) {
    context.current_state.current_state = IR_BARCODE & IR_MASK;
}

//detect line loss
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

void ir_get_telemetry(ir_subsystem_t *out_snapshot) {

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