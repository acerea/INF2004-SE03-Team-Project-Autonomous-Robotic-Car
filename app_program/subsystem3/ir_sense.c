#include <stdint.h>
#include "ir_sense.h"
#include <tk/tkernel.h>
#include <stdbool.h>
#include <bsp/libbsp.h>
#include <sys/sysdef.h>
//WHAT IS GOING ON MAN WHY DID I WRITE DUPE INCLUDES
#define PIN_IR_LEFT 26
#define PIN_IR_CENTRE 27
#define PIN_IR_RIGHT 28

#define MAX_PULSES       64
#define BARCODE_END   150000
#define TIMEOUTLOSS      2000

#define IRQ_IO_BANK0          13
#define IO_BANK0_INTR_BASE    (IO_BANK0_BASE + 0x0F0)
#define IO_BANK0_PROC0_INTE   (IO_BANK0_BASE + 0x100)
#define IO_BANK0_PROC0_INTS   (IO_BANK0_BASE + 0x120)
#define STATE_TABLE_SIZE (sizeof(state_execution_table) / sizeof(state_execution_table[0]))

#define GPIO_EDGE_FALL_BIT(pin) (1U << (((pin) % 8) * 4 + 2))
#define GPIO_EDGE_RISE_BIT(pin) (1U << (((pin) % 8) * 4 + 3))
#define GPIO_INTR_REG_IDX(pin)  ((pin) / 8)

static buddy3_context_t context;
static ID ir_mutex_id;
static volatile bool steering_locked = false;

static volatile uint32_t barcode_pulses[MAX_PULSES];
static volatile uint8_t  barcode_levels[MAX_PULSES];
static volatile uint32_t pulse_count = 0;
static volatile uint64_t last_edge_us = 0;

typedef void (*state_handler_t)(void);

/*gpio*/
static inline void ir_gpio_init(UINT pin)
{
    out_w(GPIO_OE_CLR, (1u << pin));
    out_w(GPIO_CTRL(pin), 5u); 
}

static inline bool ir_gpio_get(UINT pin)
{
    return (in_w(GPIO_IN) & (1u << pin)) != 0u;
}

static inline uint64_t ir_time_us_64(void)
{
    uint32_t lo = in_w(TIMER_BASE + 0x28u); 
    uint32_t hi = in_w(TIMER_BASE + 0x24u); 
    return ((uint64_t)hi << 32) | lo;
}

static void ir_gpio_irq_enable(UINT pin, bool enable)
{
    uint32_t idx       = GPIO_INTR_REG_IDX(pin);
    uint32_t mask      = GPIO_EDGE_RISE_BIT(pin) | GPIO_EDGE_FALL_BIT(pin);
    uint32_t intr_addr = IO_BANK0_INTR_BASE + (idx * 4u);
    uint32_t inte_addr = IO_BANK0_PROC0_INTE + (idx * 4u);

    out_w(intr_addr, mask); /* Clear pending edge flags */

    if (enable) {
        set_w(inte_addr, mask);
    } else {
        clr_w(inte_addr, mask);
    }
}

/*preparation*/
//get current system time in ms
static uint32_t get_time_ms(void) {
    SYSTIM tim;
    tk_get_tim(&tim);
    return (uint32_t)tim.lo;
}

//sensor calibration 
static void execute_cali(void) {
    tk_loc_mtx(ir_mutex_id, TMO_FEVR);
    context.current_state.current_state = IR_TRACK;
    tk_unl_mtx(ir_mutex_id);
}

/*line following and estimation*/
static void execute_track(void) {
    bool left_on_line = ir_gpio_get(PIN_IR_LEFT);
    bool centre_on_line = ir_gpio_get(PIN_IR_CENTRE);
    bool right_on_line = ir_gpio_get(PIN_IR_RIGHT);

    tk_loc_mtx(ir_mutex_id, TMO_FEVR);

    context.current_state.current_state = IR_TRACK & IR_MASK;

    if (steering_locked) {
        tk_unl_mtx(ir_mutex_id);
        return;
    }

    // Traditional line-loss fallback logic (all sensors read off-line)
    if (!left_on_line && !centre_on_line && !right_on_line) {
        context.current_state.current_state = IR_LINE_LOST;
        context.line_lost_start_ms = get_time_ms();
    }

    tk_unl_mtx(ir_mutex_id);
}

//detect line loss and response
static void execute_line_lost(void) {

    uint32_t now = get_time_ms();

    tk_loc_mtx(ir_mutex_id, TMO_FEVR);

    context.current_state.current_state = IR_LINE_LOST & IR_MASK;

    if (context.line_lost_start_ms == 0) {
        context.line_lost_start_ms = now;
    }

    if ((now - context.line_lost_start_ms) > TIMEOUTLOSS) {
        context.emergency_stop_active = true;
    }

    tk_unl_mtx(ir_mutex_id);

}

/*barcode task*/
static void barcode_gpio_isr(UINT intnum) {
    uint32_t idx       = GPIO_INTR_REG_IDX(PIN_IR_RIGHT);
    uint32_t fall_mask = GPIO_EDGE_FALL_BIT(PIN_IR_RIGHT);
    uint32_t rise_mask = GPIO_EDGE_RISE_BIT(PIN_IR_RIGHT);
    uint32_t both_mask = fall_mask | rise_mask;

    uint32_t status = in_w(IO_BANK0_PROC0_INTS + (idx * 4u)) & both_mask;
    if (status == 0u) {
        return;
    }

    out_w(IO_BANK0_INTR_BASE + (idx * 4u), status);

    uint64_t now = ir_time_us_64();

    if (last_edge_us != 0 && pulse_count < MAX_PULSES) {
        barcode_pulses[pulse_count] = (uint32_t)(now - last_edge_us);
        // If falling edge (1 -> 0), the bar that just finished was HIGH (Black)
        barcode_levels[pulse_count] = (status & fall_mask) ? 1u : 0u;
        pulse_count++;
    }
    last_edge_us = now;
}

//transition into Barcode mode from execute_track()
static void start_barcode_scan(void) {
    pulse_count = 0;
    last_edge_us = 0;

    tk_loc_mtx(ir_mutex_id, TMO_FEVR);
    context.current_state.current_state = IR_BARCODE & IR_MASK;
    context.barcode_started = true;
    context.pulse_count = 0;
    context.string_index = 0;
    context.decoded_string[0] = '\0';
    tk_unl_mtx(ir_mutex_id);

    ir_locksteer(true);
    ir_gpio_irq_enable(PIN_IR_RIGHT, true);

}

static char decode_pattern_9bit(uint16_t pattern)
{
    for (size_t i = 0; i < LOOKUP_SIZE; i++) {
        if (nav_letters[i].pattern == pattern) {
            return nav_letters[i].alphabet;
        }
    }
    return '?';
}
//called by state_execution_table[IR_BARCODE]
static void execute_barcode(void) {
    uint64_t now = ir_time_us_64();

    // Wait until pulses are captured and a timeout gap indicates the robot cleared the barcode
    if (pulse_count > 0 && (now - last_edge_us) > BARCODE_END) {

        uint32_t count = pulse_count;
        uint32_t min_w = UINT32_MAX;
        uint32_t max_w = 0;

        // 1. Find narrowest and widest bars to adapt to robot driving speed
        for (uint32_t i = 0; i < count; i++) {
            if (barcode_pulses[i] < min_w) min_w = barcode_pulses[i];
            if (barcode_pulses[i] > max_w) max_w = barcode_pulses[i];
        }

        // Dynamic threshold between narrow (0) and wide (1) bars
        uint32_t threshold_us = (min_w + max_w) / 2;

        tk_loc_mtx(ir_mutex_id, TMO_FEVR);
        context.pulse_count = count;

        // Convert pulse widths into '0' (narrow) and '1' (wide) binary stream
        uint32_t limit = (count < sizeof(context.decoded_string) - 1) ? count : (sizeof(context.decoded_string) - 1);
        for (uint32_t i = 0; i < limit; i++) {
            context.decoded_string[i] = (barcode_pulses[i] > threshold_us) ? '1' : '0';
        }
        context.decoded_string[limit] = '\0';
        context.string_index = limit;

        // Return to normal line tracking
        context.barcode_started = false;
        context.current_state.current_state = IR_TRACK & IR_MASK;
        tk_unl_mtx(ir_mutex_id);

        ir_locksteer(false);
    }
}



/*handling*/
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
void ir_locksteer(bool lock) {

    steering_locked = lock;
}



//ir sensor task core 0
static void ir_sensor_task(INT stacd, void *exinf) {

    while (1) {
        tk_loc_mtx(ir_mutex_id, TMO_FEVR);
        uint8_t target_state = context.current_state.current_state & IR_MASK;
        tk_unl_mtx(ir_mutex_id);

        if (target_state < STATE_TABLE_SIZE && state_execution_table[target_state] != NULL) {
            state_execution_table[target_state]();
        }

        tk_loc_mtx(ir_mutex_id, TMO_FEVR);
        context.current_state.timestamp = get_time_ms();
        tk_unl_mtx(ir_mutex_id);

        tk_dly_tsk(10);
    }
}


//configurations
void ir_sense_init(void) {

    ir_gpio_init(PIN_IR_LEFT);
    gpio_set_pin(PIN_IR_LEFT, GPIO_IN);

    ir_gpio_init(PIN_IR_CENTRE);
    gpio_set_pin(PIN_IR_CENTRE, GPIO_IN);

    ir_gpio_init(PIN_IR_RIGHT);
    gpio_set_pin(PIN_IR_RIGHT, GPIO_IN);

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