#ifndef IR_SENSE_H
#include <stdbool.h>

#define IR_MASK 0x03 /**< Bitmask to isolate 2-bit command data        */
#define IR_CALI 0x00
#define IR_TRACK 0x01
#define IR_BARCODE 0x02
#define IR_LINE_LOST 0x03

typedef enum {
    NAV_CMD_LEFT     = 0x00,    /**< 00: Execute left turn at next junction      */
    NAV_CMD_RIGHT    = 0x01,    /**< 01: Execute right turn at next junction     */
    NAV_CMD_STRAIGHT = 0x02,    /**< 10: Execute straight movement at junction  */
    NAV_CMD_UTURN    = 0x03     /**< 11: Execute U-turn maneuver at junction    */
} nav_cmd_t;

typedef enum {
    uint32_t timestamp;
    int16_t estimated_line_pos;
    uint8_t junction_detected;
    uint8_t current_state;
} ir_subsystem_state_t;

typedef struct {
    ir_subsystem_state_t current_state;
    
    /* Sensor Thresholding Context */
    uint16_t threshold_black;
    uint16_t threshold_white;
    
    /* Barcode Decoder Status */
    uint32_t edge_intervals[16];
    uint8_t edge_count;
    uint8_t barcode_retry_count;
    uint8_t pending_raw_payload; /**< Bit-packed 2-bit navigation payload    */
    
    /* Line Lost Safety Monitoring */
    uint32_t line_lost_start_ms;
    bool emergency_stop_active;
} buddy3_context_t;

/*system integration*/
void ir_sense_init(void);
void ir_sense_start(void);
void ir_locksteer(bool lock);
void ir_get_telemetry(ir_subsystem_state_t *out_snapshot);

#endif /* IR_SENSE_H */
