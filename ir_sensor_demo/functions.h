// functions.h
#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <stdbool.h>
#include <stdint.h>
#include "pico/stdlib.h"

// ---------- Pin/Channel map (RP2040) ----------
// LINE sensor (center): GP26 = ADC0
#define LINE_ADC_GPIO           27
#define LINE_ADC_CHANNEL        1

// BARCODE analog (thresholding / trigger): GP28 = ADC2
#define BARCODE_ADC_GPIO        28
#define BARCODE_ADC_CHANNEL     2

// BARCODE digital (pulse width): GP7
#define BARCODE_DIGITAL_PIN      7

// ---------- Line calibration struct ----------
typedef struct {
    uint16_t white_val;
    uint16_t black_val;
    uint16_t low_thresh;
    uint16_t high_thresh;
    uint16_t span;
    bool     white_is_higher;
} LineCalib;

// ---------- General init ----------
void setup_dual_sensors(void);

// ---------- ADC helpers ----------
uint16_t read_adc_channel(int channel, int samples);

// Decide WHITE/BLACK using hysteresis thresholds
static inline bool ir_detect_surface(uint16_t raw, bool prev_is_white, const LineCalib* cal) {
    if (prev_is_white) {
        // need go darker than low_thresh to flip to BLACK
        return (raw > cal->low_thresh); // still WHITE?
    } else {
        // need go brighter than high_thresh to flip to WHITE
        return (raw >= cal->high_thresh); // now WHITE?
    }
}

// ---------- Time helpers ----------
static inline absolute_time_t custom_delayed_by_ms(absolute_time_t t, uint32_t ms) {
    return delayed_by_us(t, (int64_t)ms * 1000);
}

// ===================================================
//                BARCODE (Code 39 style)
// ===================================================

#define MAX_BARCODE_SEGMENTS    128   // plenty for a few symbols while driving
#define BARCODE_ARM_MS           60   // time to qualify initial black (also used by main)
#define BARCODE_CAPTURE_TIMEOUT_MS 900
#define BARCODE_END_WHITE_MS    120   // white hold meaning "barcode ended"

void barcode_init(uint16_t analog_threshold, bool white_is_higher);

// Quick analog check (used for trigger/arming)
bool barcode_is_white(void);

// Trigger gate helper (uses analog)
bool barcode_trigger_ready(void);

// Start / Update / Query capture (uses GP7 ISR)
void barcode_capture_start(void);
// returns true when capture finished (either success or timeout)
bool barcode_capture_update(void);
void barcode_capture_abort(void);
uint32_t barcode_capture_segment_count(void);
bool barcode_capture_is_active(void);

// Decode last captured pulses to a Code39 letter ('A'..'Z') or '?' if fail
char barcode_decode_last(void);

// Debug: Print captured pulse information (only in debug version)
void barcode_print_capture_debug(void);

// Cooldown utility for main FSM
void barcode_set_cooldown_ms(uint32_t ms);
bool barcode_in_cooldown(void);

#endif // FUNCTIONS_H
