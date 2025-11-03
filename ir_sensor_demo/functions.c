// functions.c - WITH ENHANCED BARCODE DIAGNOSTICS
#include "functions.h"
#include <string.h>
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "pico/time.h"
#include <stdio.h>

// Forward declaration of ISR used during setup
static void barcode_edge_isr(uint gpio, uint32_t events);

// =================== General init ===================
void setup_dual_sensors(void) {
    stdio_init_all();

    // ADC init
    adc_init();
    // LINE = ADC0 (GP26)
    adc_gpio_init(LINE_ADC_GPIO);
    // BARCODE analog = ADC2 (GP28)
    adc_gpio_init(BARCODE_ADC_GPIO);

    // BARCODE digital
    gpio_init(BARCODE_DIGITAL_PIN);
    gpio_set_dir(BARCODE_DIGITAL_PIN, GPIO_IN);
    gpio_pull_up(BARCODE_DIGITAL_PIN); // many modules are open-collector; safe default

    // install ISR callback once
    gpio_set_irq_enabled_with_callback(
        BARCODE_DIGITAL_PIN,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        &barcode_edge_isr
    );
    // immediately disable until we're ready to capture
    gpio_set_irq_enabled(BARCODE_DIGITAL_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);

}

// =================== ADC helpers ===================
uint16_t read_adc_channel(int channel, int samples) {
    adc_select_input(channel);
    uint32_t acc = 0;
    for (int i = 0; i < samples; ++i) {
        acc += adc_read();
        tight_loop_contents();
    }
    return (uint16_t)(acc / (uint32_t)(samples > 0 ? samples : 1));
}

// ===================================================
//                 BARCODE (Code 39 style)
// ===================================================

// ---- Shared analog threshold for quick checks ----
static uint16_t s_bc_thresh = 0;
static bool     s_bc_white_high = true;

// Quick analog read → WHITE/BLACK
bool barcode_is_white(void) {
    uint16_t raw = read_adc_channel(BARCODE_ADC_CHANNEL, 4);
    return s_bc_white_high ? (raw > s_bc_thresh) : (raw < s_bc_thresh);
}

// Trigger: see BLACK for BARCODE_ARM_MS while not cooling down
static absolute_time_t s_bc_black_start = {0};
static absolute_time_t s_bc_cooldown_until = {0};

bool barcode_in_cooldown(void) {
    // FIXED: Check if cooldown_until is in the future
    return absolute_time_diff_us(get_absolute_time(), s_bc_cooldown_until) > 0;
}
void barcode_set_cooldown_ms(uint32_t ms) {
    s_bc_cooldown_until = delayed_by_ms(get_absolute_time(), ms);
}

bool barcode_trigger_ready(void) {
    if (barcode_in_cooldown()) return false;

    bool white = barcode_is_white();
    if (!white) {
        if (to_us_since_boot(s_bc_black_start) == 0) {
            s_bc_black_start = get_absolute_time();
        } else {
            int64_t held = absolute_time_diff_us(s_bc_black_start, get_absolute_time());
            if (held >= (int64_t)BARCODE_ARM_MS * 1000) {
                s_bc_black_start = (absolute_time_t){0};
                return true;
            }
        }
    } else {
        s_bc_black_start = (absolute_time_t){0};
    }
    return false;
}

void barcode_init(uint16_t analog_threshold, bool white_is_higher) {
    s_bc_thresh = analog_threshold;
    s_bc_white_high = white_is_higher;

    // Prepare ISR for digital pulse timing
    gpio_set_irq_enabled(BARCODE_DIGITAL_PIN, 0, false);
}

// -------------- Digital pulse capture (ISR) --------------
static volatile bool  cap_active = false;
static volatile bool  cap_done   = false;
static volatile uint32_t seg_count = 0;
static volatile uint32_t seg_ms[MAX_BARCODE_SEGMENTS];
static volatile absolute_time_t last_edge_time;
static volatile bool last_level = false; // false = low(black), true = high(white)
static volatile absolute_time_t cap_start_time;

static void barcode_edge_isr(uint gpio, uint32_t events) {
    if (!cap_active) return;
    absolute_time_t now = get_absolute_time();

    bool level = gpio_get(BARCODE_DIGITAL_PIN); // true=HIGH(white), false=LOW(black)

    // First edge handling
    if (to_us_since_boot(last_edge_time) == 0) {
        last_edge_time = now;
        last_level = level;
        return;
    }

    // Pulse duration since last transition
    uint32_t dur_ms = (uint32_t)(absolute_time_diff_us(last_edge_time, now) / 1000);
    last_edge_time = now;

    // Store
    if (seg_count < MAX_BARCODE_SEGMENTS) {
        seg_ms[seg_count++] = dur_ms;
    } else {
        // Overflow: mark done; main will timeout
        cap_done = true;
        cap_active = false;
    }

    last_level = level;
}

void barcode_capture_start(void) {
    memset((void*)seg_ms, 0, sizeof(seg_ms));
    seg_count = 0;
    cap_done = false;
    cap_active = true;
    cap_start_time = get_absolute_time();
    last_edge_time = (absolute_time_t){0};

    // Clear any pending and arm ISR on both edges
    // gpio_set_irq_enabled_with_callback(BARCODE_DIGITAL_PIN,
    //     GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, barcode_edge_isr);

    gpio_acknowledge_irq(BARCODE_DIGITAL_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
    gpio_set_irq_enabled(BARCODE_DIGITAL_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

// Complete when: timeout OR long white after some segments
bool barcode_capture_update(void) {
    if (!cap_active && cap_done) return true; // already finished

    // timeout
    if (absolute_time_diff_us(cap_start_time, get_absolute_time()) >=
        (int64_t)BARCODE_CAPTURE_TIMEOUT_MS * 1000) {
        cap_active = false;
        cap_done = true;
        gpio_set_irq_enabled(BARCODE_DIGITAL_PIN, 0, false);
        return true;
    }

    // End condition: after a few transitions, hold white long enough without edges.
    if (seg_count >= 6) {
        // Check how long since last edge
        if (to_us_since_boot(last_edge_time) != 0) {
            uint32_t idle_ms = (uint32_t)(absolute_time_diff_us(last_edge_time, get_absolute_time()) / 1000);
            if (idle_ms >= BARCODE_END_WHITE_MS && barcode_is_white()) {
                cap_active = false;
                cap_done = true;
                gpio_set_irq_enabled(BARCODE_DIGITAL_PIN, 0, false);
                return true;
            }
        }
    }

    return false;
}

// *** NEW: Debug function to print captured pulses ***
void barcode_print_capture_debug(void) {
    printf("\n=== BARCODE CAPTURE DEBUG ===\n");
    printf("Segments captured: %lu\n", (unsigned long)seg_count);
    
    if (seg_count == 0) {
        printf("NO PULSES CAPTURED!\n");
        printf("Possible issues:\n");
        printf("  - Digital pin (GP7) not connected\n");
        printf("  - Barcode sensor not outputting digital signal\n");
        printf("  - Driving too fast over barcode\n");
        printf("=============================\n\n");
        return;
    }
    
    printf("Pulse durations (ms):\n");
    for (uint32_t i = 0; i < seg_count && i < 32; i++) {
        printf("  [%2lu] %4lums", (unsigned long)i, (unsigned long)seg_ms[i]);
        if (i % 4 == 3) printf("\n");
    }
    if (seg_count % 4 != 0) printf("\n");
    
    if (seg_count > 32) {
        printf("  ... (%lu more segments)\n", (unsigned long)(seg_count - 32));
    }
    
    // Statistics
    uint32_t sum = 0;
    uint32_t min_dur = 0xFFFFFFFF;
    uint32_t max_dur = 0;
    for (uint32_t i = 0; i < seg_count; i++) {
        sum += seg_ms[i];
        if (seg_ms[i] < min_dur) min_dur = seg_ms[i];
        if (seg_ms[i] > max_dur) max_dur = seg_ms[i];
    }
    uint32_t avg = seg_count > 0 ? sum / seg_count : 0;
    
    printf("\nStats: min=%lums, max=%lums, avg=%lums\n", 
           (unsigned long)min_dur, (unsigned long)max_dur, (unsigned long)avg);
    printf("Ratio: max/min = %.2f (Code39 expects ~2.5-3.0)\n", 
           min_dur > 0 ? (float)max_dur / (float)min_dur : 0.0f);
    printf("=============================\n\n");
}

// --------- Code 39 decoding (letters only) ---------
// Map characters A..Z to 9-bit wide-mask (1=wide, 0=narrow) for bar/space sequence.
// Order is: bar, space, bar, space, bar, space, bar, space, bar.
// (Patterns from Code 39 spec; only letters here.)
typedef struct { char c; uint16_t mask9; } C39Lut;
static const C39Lut LUT[] = {
    //   c   mask (bit8..bit0). Example for 'A' = 100001001 (bars: wide-narrow..)
    {'A', 0b100001001},
    {'B', 0b001001001},
    {'C', 0b101001000},
    {'D', 0b000101001},
    {'E', 0b100101000},
    {'F', 0b001101000},
    {'G', 0b000001101},
    {'H', 0b100001100},
    {'I', 0b001001100},
    {'J', 0b000101100},
    {'K', 0b100000011},
    {'L', 0b001000011},
    {'M', 0b101000010},
    {'N', 0b000100011},
    {'O', 0b100100010},
    {'P', 0b001100010},
    {'Q', 0b000000111},
    {'R', 0b100000110},
    {'S', 0b001000110},
    {'T', 0b000100110},
    {'U', 0b110000001},
    {'V', 0b011000001},
    {'W', 0b111000000},
    {'X', 0b010100001},
    {'Y', 0b110100000},
    {'Z', 0b011100000},
};

// Convert durations → relative units (narrow/wide) with robust threshold
static int classify_narrow_wide(const uint32_t *dur_ms, uint32_t n, uint8_t *out_bits /*size ≥ n*/) {
    if (n == 0) return 0;
    // Compute median
    uint32_t tmp[32];
    uint32_t m = n > 32 ? 32 : n;
    for (uint32_t i=0;i<m;i++) tmp[i] = dur_ms[i];
    // simple selection sort for small m
    for (uint32_t i=0;i<m;i++){
        for(uint32_t j=i+1;j<m;j++){
            if(tmp[j]<tmp[i]){ uint32_t t=tmp[i]; tmp[i]=tmp[j]; tmp[j]=t; }
        }
    }
    uint32_t median = tmp[m/2]; if (median == 0) median = 1;

    // Threshold between narrow and wide ~ 1.8× median (Code39 wide≈2.5× narrow; pick conservative)
    float thr = median * 1.8f;

    printf("[DECODE] Median pulse: %lums, threshold: %.1fms\n", 
           (unsigned long)median, thr);
    printf("[DECODE] Classification: ");
    for (uint32_t i=0;i<n;i++) {
        out_bits[i] = (dur_ms[i] > (uint32_t)thr) ? 1 : 0;
        printf("%c", out_bits[i] ? 'W' : 'n');
    }
    printf("\n");

    return (int)n;
}

// Try to find a single 9-element symbol in the captured stream (ignore start/stop)
char barcode_decode_last(void) {
    // First print debug info
    barcode_print_capture_debug();
    
    // Copy volatile buffer
    uint32_t n = seg_count;
    if (n < 9) {
        printf("[DECODE] Too few segments (%lu < 9)\n", (unsigned long)n);
        return '?';
    }
    if (n > MAX_BARCODE_SEGMENTS) n = MAX_BARCODE_SEGMENTS;

    uint32_t d[MAX_BARCODE_SEGMENTS];
    for (uint32_t i=0;i<n;i++) d[i] = seg_ms[i];

    // The capture begins at some edge; Code39 symbol is 9 elements (bar/space alternate).
    // Try sliding window of 9 with parity (bar first). We can try both offsets 0 and 1.
    uint8_t bits[16];

    printf("[DECODE] Trying %lu possible windows...\n", (unsigned long)(n >= 9 ? n - 8 : 0));

    for (uint32_t offset = 0; offset + 9 <= n; ++offset) {
        printf("[DECODE] Window at offset %lu: ", (unsigned long)offset);
        
        // Classify widths on this 9-seg window
        classify_narrow_wide(&d[offset], 9, bits);

        // Build mask
        uint16_t mask = 0;
        for (int i=0;i<9;i++) { mask = (uint16_t)((mask<<1) | (bits[i] ? 1 : 0)); }

        printf("[DECODE] Mask: 0b%09b (0x%03X)\n", mask, mask);

        // Compare to LUT
        for (uint32_t k=0; k < sizeof(LUT)/sizeof(LUT[0]); ++k) {
            if (mask == LUT[k].mask9) {
                printf("[DECODE] *** MATCH FOUND: '%c' ***\n", LUT[k].c);
                return LUT[k].c;
            }
        }
    }

    printf("[DECODE] No match found in any window\n");
    return '?';
}

void barcode_capture_abort(void) {
    cap_active = false;
    cap_done = false;
    seg_count = 0;
    last_edge_time = (absolute_time_t){0};
    gpio_set_irq_enabled(BARCODE_DIGITAL_PIN,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
}

uint32_t barcode_capture_segment_count(void) {
    return seg_count;
}

bool barcode_capture_is_active(void) {
    return cap_active;
}
