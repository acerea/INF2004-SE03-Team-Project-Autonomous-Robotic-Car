#ifndef IR_SENSE_H
#define IR_SENSE_H
#include <stdbool.h>
#include <stdint.h>

#define IR_MASK 0x03 //< Bitmask to isolate 2-bit command data
#define IR_CALI 0x00
#define IR_TRACK 0x01
#define IR_BARCODE 0x02
#define IR_LINE_LOST 0x03


//navigation commands w lookup table
typedef struct {
    uint16_t pattern;
    char alphabet;
} barcode_lu;

static const barcode_lu nav_letters[] = { //lookup table for alphabets
    { 0b100001001, 'A' }, { 0b001001001, 'B' }, { 0b101001000, 'C' }, { 0b000011001, 'D' }, { 0b100011000, 'E' }, 
    { 0b001011000, 'F' }, { 0b000001101, 'G' }, { 0b100001100, 'H' }, { 0b001001100, 'I' }, { 0b000011100, 'J' }, 
    { 0b100000011, 'K' }, { 0b001000011, 'L' }, { 0b101000010, 'M' }, { 0b000010011, 'N' }, { 0b100010010, 'O' }, 
    { 0b001010010, 'P' }, { 0b000000111, 'Q' }, { 0b100000110, 'R' }, { 0b001000110, 'S' }, { 0b000010110, 'T' }, 
    { 0b110000001, 'U' }, { 0b011000001, 'V' }, { 0b111000000, 'W' }, { 0b010010001, 'X' }, { 0b110010000, 'Y' }, { 0b011010000, 'Z' }
};

#define LOOKUP_SIZE (sizeof(nav_letters)/sizeof(barcode_lu))

//realtime subsys snapshot
typedef struct {
    uint32_t timestamp;
    int16_t estimated_line_pos;
    uint8_t junction_detected;
    uint8_t current_state;
    
} ir_subsystem_state_t;

//subsys tracking
typedef struct {
    ir_subsystem_state_t current_state;
    
    /* Line Lost Safety Monitoring */
    uint32_t line_lost_start_ms;
    bool emergency_stop_active;

    //non-blocking barcode parsing
    uint16_t pulse_widths[9];
    uint8_t pulse_count; 
    bool barcode_started; 

    //navigation generatino buffer
    char decoded_string[16]; 
    uint8_t string_index;

} buddy3_context_t;

// system integration
void ir_sense_init(void);
void ir_sense_start(void);
void ir_locksteer(bool lock);
void ir_get_telemetry(ir_subsystem_state_t *out_snapshot);

#endif // IR_SENSE_H
