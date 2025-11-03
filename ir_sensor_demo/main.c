// main.c - IMPROVED BARCODE DETECTION
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "functions.h"
#include "motor_control.h"
#include <stdio.h>
#include <string.h>

// ===================== TUNING =====================
#define LOOP_DT_MS               5

// Base drive speeds
#define SPEED_STRAIGHT_PWM    28000
#define SPEED_TURN_FAST_PWM   28000
#define SPEED_TURN_SLOW_PWM    7000
#define SPEED_REACQUIRE_PWM   25000

// Barcode detection - IMPROVED TIMING
#define BARCODE_BACKUP_PWM     10000  // Slightly faster backup
#define BARCODE_BACKUP_MS       200   // Increased backup distance
#define BARCODE_CRAWL_PWM      6000   // SLOWER crawl for stable capture
#define BARCODE_SCAN_TIMEOUT_MS 6000  // Longer timeout
#define BARCODE_PRECHECK_MS     300   // Longer precheck
#define BARCODE_PRECHECK_PWM    6000  // Slower precheck
#define BARCODE_MIN_PRECHECK_SEGMENTS 4  // More segments to confirm
#define BARCODE_FALSE_TRIGGER_COOLDOWN_MS 500

// Line following
#define FOLLOW_GAIN            0.80f
#define ERROR_DEADBAND         0.03f
#define MAX_FOLLOW_ADJ        13000

// Loss/scan timings
#define LOST_WHITE_MS             80
#define PRE_NUDGE_START_MS        35
#define PRE_NUDGE_LEN_MS          35
#define SCAN_SWEEP_MS            220
#define SCAN_RECENTER_MS         140
#define REACQUIRE_MAX_MS         900
#define REACQUIRE_HOLD_MS         50
#define BLACK_CONFIRM_MS          40

// Smoothing
#define SLEW_LIM               2500
#define EMA_ALPHA              0.35f

// Calibration
#define PRESET_WHITE_VAL         200
#define PRESET_BLACK_VAL        1100

// Startup
#define STARTUP_DELAY_MS        3000
#define LED_PIN                   25

#define BC_COOLDOWN_MS           2000

// ======================================================

static inline bool is_turn_right_letter(char c) {
    if (c < 'A' || c > 'Z') return false;
    int idx = c - 'A';
    return (idx % 2 == 0);
}

typedef enum {
    ST_FOLLOW = 0,
    ST_LOST_WAIT,
    ST_SCAN_LEFT,
    ST_SCAN_RECENTER,
    ST_SCAN_RIGHT,
    ST_TURN_REACQUIRE,
    ST_BC_PRECHECK,
    ST_BC_TRIGGERED,
    ST_BC_POSITION,
    ST_BC_SCAN,
    ST_BC_DECODE
} state_t;

static LineCalib g_cal;
static int last_dir = +1;

// ===================== LED =====================
static inline void led_init(void){ 
    gpio_init(LED_PIN); 
    gpio_set_dir(LED_PIN, GPIO_OUT); 
}
static inline void led_on(void){ gpio_put(LED_PIN, 1); }
static inline void led_off(void){ gpio_put(LED_PIN, 0); }
static inline void led_blink(uint32_t n, uint32_t d){
    for(uint32_t i=0;i<n;i++){ 
        led_on(); 
        sleep_ms(d); 
        led_off(); 
        sleep_ms(d); 
    }
}

// ===================== MOTORS =====================
static inline void drive_straight(uint16_t pwm) {
    start_motor(MOTOR_LEFT,  FORWARD, pwm);
    start_motor(MOTOR_RIGHT, FORWARD, pwm);
}
static inline void drive_backward(uint16_t pwm) {
    start_motor(MOTOR_LEFT,  BACKWARD, pwm);
    start_motor(MOTOR_RIGHT, BACKWARD, pwm);
}
static inline void swing_left(uint16_t fast_pwm, uint16_t slow_pwm) {
    start_motor(MOTOR_LEFT,  FORWARD, slow_pwm);
    start_motor(MOTOR_RIGHT, FORWARD, fast_pwm);
}
static inline void swing_right(uint16_t fast_pwm, uint16_t slow_pwm) {
    start_motor(MOTOR_LEFT,  FORWARD, fast_pwm);
    start_motor(MOTOR_RIGHT, FORWARD, slow_pwm);
}
static inline void stop_now(void) {
    start_motor(MOTOR_LEFT,  FORWARD, 0);
    start_motor(MOTOR_RIGHT, FORWARD, 0);
}

// ========== Smoothing ==========
static inline uint16_t slew(uint16_t prev, int32_t target, int32_t step) {
    int32_t p = (int32_t)prev;
    if (target > p + step) target = p + step;
    else if (target < p - step) target = p - step;
    if (target < 0) target = 0;
    if (target > 65535) target = 65535;
    return (uint16_t)target;
}

static inline int32_t follow_adjust(float norm) {
    float err = (norm - 0.5f) * 2.0f;
    float mag = (err >= 0.0f) ? err : -err;
    if (mag < ERROR_DEADBAND) mag = 0.0f;
    if (mag > 1.0f) mag = 1.0f;
    int sgn = last_dir;
    int32_t adj = (int32_t)(sgn * (mag * FOLLOW_GAIN) * (float)MAX_FOLLOW_ADJ);
    if (adj >  MAX_FOLLOW_ADJ) adj =  MAX_FOLLOW_ADJ;
    if (adj < -MAX_FOLLOW_ADJ) adj = -MAX_FOLLOW_ADJ;
    return adj;
}

// ===================== MAIN =====================
int main() {
    stdio_init_all();
    led_init();

    setup_dual_sensors();
    setup_motor();
    setup_encoders();

    printf("\n");
    printf("===========================================\n");
    printf("    BARCODE LINE FOLLOWER - ENHANCED\n");
    printf("===========================================\n");
    printf("Barcode crawl speed: %d PWM\n", BARCODE_CRAWL_PWM);
    printf("Barcode precheck: %d ms at %d PWM\n", BARCODE_PRECHECK_MS, BARCODE_PRECHECK_PWM);
    printf("===========================================\n\n");

    // ----- Calibration -----
    if (PRESET_WHITE_VAL == 0 || PRESET_BLACK_VAL == 0) {
        printf("Place LINE sensor on WHITE and press Enter...\n");
        while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {}
        while (getchar_timeout_us(1000000) == PICO_ERROR_TIMEOUT) {}
        uint16_t w = read_adc_channel(LINE_ADC_CHANNEL, 100);

        printf("Place LINE sensor on BLACK and press Enter...\n");
        while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {}
        while (getchar_timeout_us(1000000) == PICO_ERROR_TIMEOUT) {}
        uint16_t b = read_adc_channel(LINE_ADC_CHANNEL, 100);

        g_cal.white_val = w;
        g_cal.black_val = b;
    } else {
        g_cal.white_val = PRESET_WHITE_VAL;
        g_cal.black_val = PRESET_BLACK_VAL;
    }

    g_cal.white_is_higher = (g_cal.white_val > g_cal.black_val);
    g_cal.span  = (g_cal.white_val > g_cal.black_val)
                ? (g_cal.white_val - g_cal.black_val)
                : (g_cal.black_val - g_cal.white_val);
    if (g_cal.span == 0) g_cal.span = 1;
    
    uint16_t mid = (g_cal.white_val + g_cal.black_val)/2;
    uint16_t hyst = (g_cal.span * 6) / 100; 
    if (hyst < 1) hyst = 1;
    g_cal.low_thresh  = (uint16_t)(mid - (hyst/2));
    g_cal.high_thresh = (uint16_t)(mid + (hyst/2));

    printf("Line calibration:\n");
    printf("  White: %u, Black: %u\n", g_cal.white_val, g_cal.black_val);
    printf("  Thresholds: %u / %u\n\n", g_cal.low_thresh, g_cal.high_thresh);

    // ----- Barcode init -----
    uint16_t bc_thresh = (uint16_t)((g_cal.low_thresh + g_cal.high_thresh)/2);
    barcode_init(bc_thresh, false);  // Assuming black bar < white space

    // Test digital pin
    printf("Testing barcode digital pin (GP7)...\n");
    bool digital_state = gpio_get(BARCODE_DIGITAL_PIN);
    uint16_t bc_analog = read_adc_channel(BARCODE_ADC_CHANNEL, 20);
    printf("  Digital: %s, Analog: %u\n", digital_state ? "HIGH" : "LOW", bc_analog);
    if (!digital_state) {
        printf("  WARNING: Digital pin reads LOW (expected HIGH with pull-up)\n");
        printf("  Check sensor connection and power!\n");
    }
    printf("\n");

    printf("Starting in %d s...\n", STARTUP_DELAY_MS/1000);
    for (int s = STARTUP_DELAY_MS/1000; s>0; --s) {
        printf("%d...\n", s);
        led_blink(1, 80);
        sleep_ms(920);
    }
    printf("GO!\n\n");
    led_blink(3, 80);

    // FSM state
    state_t st = ST_FOLLOW;
    absolute_time_t t_last_nonwhite = get_absolute_time();
    absolute_time_t t_state_start   = get_absolute_time();
    uint16_t scan_min_left = 0xFFFF, scan_min_right = 0xFFFF;
    uint32_t left_hold = 0, right_hold = 0;

    uint16_t prevL = 0, prevR = 0;
    float ema = (float)read_adc_channel(LINE_ADC_CHANNEL, 8);
    uint32_t loop_count = 0;
    uint32_t barcode_attempts = 0;

    barcode_set_cooldown_ms(BC_COOLDOWN_MS);

    while (true) {
        sleep_ms(LOOP_DT_MS);

        // ---------- Barcode trigger check ----------
        if (st == ST_FOLLOW && !barcode_in_cooldown()) {
            if (barcode_trigger_ready()) {
                barcode_attempts++;
                printf("\n*** BARCODE TRIGGER #%lu ***\n", (unsigned long)barcode_attempts);
                printf("Analog hold detected, starting precheck...\n");
                
                drive_straight(BARCODE_PRECHECK_PWM);
                barcode_capture_start();
                st = ST_BC_PRECHECK;
                t_state_start = get_absolute_time();
                prevL = prevR = BARCODE_PRECHECK_PWM;
                continue;
            }
        }

        // ---------- Line read / EMA ----------
        uint16_t raw = read_adc_channel(LINE_ADC_CHANNEL, 4);
        ema = (1.0f - EMA_ALPHA) * ema + EMA_ALPHA * (float)raw;
        uint16_t filt = (uint16_t)ema;

        static bool was_white = true;
        bool is_white = ir_detect_surface(filt, was_white, &g_cal);
        was_white = is_white;

        // Periodic status
        if (++loop_count % (200/LOOP_DT_MS) == 0) {
            uint16_t bc_adc = read_adc_channel(BARCODE_ADC_CHANNEL, 2);
            bool bc_dig = gpio_get(BARCODE_DIGITAL_PIN);
            const char* sname =
                (st==ST_FOLLOW)?"FOLLOW":
                (st==ST_LOST_WAIT)?"LOST":
                (st==ST_SCAN_LEFT)?"SCAN_L":
                (st==ST_SCAN_RECENTER)?"RECENTER":
                (st==ST_SCAN_RIGHT)?"SCAN_R":
                (st==ST_TURN_REACQUIRE)?"REACQ":
                (st==ST_BC_PRECHECK)?"BC_PRE":
                (st==ST_BC_TRIGGERED)?"BC_TRIG":
                (st==ST_BC_POSITION)?"BC_POS":
                (st==ST_BC_SCAN)?"BC_SCAN":"BC_DEC";
            printf("[%-8s] line=%4u BC_a=%4u BC_d=%s\n", 
                   sname, raw, bc_adc, bc_dig ? "H" : "L");
        }

        switch (st) {
            case ST_FOLLOW: {
                if (!is_white) {
                    t_last_nonwhite = get_absolute_time();

                    float norm = (float)(filt - g_cal.black_val) / (float)g_cal.span;
                    if (norm < 0) norm = 0; 
                    if (norm > 1) norm = 1;
                    int32_t adj = follow_adjust(norm);

                    int32_t targetL = (int32_t)SPEED_STRAIGHT_PWM - adj;
                    int32_t targetR = (int32_t)SPEED_STRAIGHT_PWM + adj;

                    uint16_t outL = slew(prevL, targetL, SLEW_LIM);
                    uint16_t outR = slew(prevR, targetR, SLEW_LIM);
                    prevL = outL; prevR = outR;

                    start_motor(MOTOR_LEFT,  FORWARD, outL);
                    start_motor(MOTOR_RIGHT, FORWARD, outR);
                } else {
                    uint32_t ms_since_black =
                        (uint32_t)(absolute_time_diff_us(t_last_nonwhite, get_absolute_time())/1000);

                    if (ms_since_black > PRE_NUDGE_START_MS && ms_since_black < LOST_WHITE_MS) {
                        if (last_dir < 0) swing_left(SPEED_TURN_FAST_PWM, SPEED_TURN_SLOW_PWM);
                        else              swing_right(SPEED_TURN_FAST_PWM, SPEED_TURN_SLOW_PWM);
                        sleep_ms(PRE_NUDGE_LEN_MS);
                        break;
                    }
                    if (ms_since_black >= LOST_WHITE_MS) {
                        stop_now();
                        t_state_start = get_absolute_time();
                        scan_min_left=scan_min_right=0xFFFF;
                        left_hold=right_hold=0;
                        st = ST_LOST_WAIT;
                    } else {
                        if (last_dir < 0) swing_left(SPEED_TURN_FAST_PWM, SPEED_TURN_SLOW_PWM);
                        else              swing_right(SPEED_TURN_FAST_PWM, SPEED_TURN_SLOW_PWM);
                    }
                }
            } break;

            case ST_BC_PRECHECK: {
                drive_straight(BARCODE_PRECHECK_PWM);

                // Ensure capture is active
                if (!barcode_capture_is_active()) {
                    printf("[BC_PRECHK] Restarting capture\n");
                    barcode_capture_start();
                }

                uint32_t segs = barcode_capture_segment_count();
                
                // Check for sufficient edges
                if (segs >= BARCODE_MIN_PRECHECK_SEGMENTS) {
                    printf("[BC_PRECHK] Digital pulses confirmed (%lu edges)\n", 
                           (unsigned long)segs);
                    barcode_capture_abort();
                    stop_now();
                    prevL = prevR = 0;
                    st = ST_BC_TRIGGERED;
                    t_state_start = get_absolute_time();
                    goto next_iter;
                }

                // Timeout or returned to white
                int64_t elapsed_ms = absolute_time_diff_us(t_state_start, get_absolute_time())/1000;
                if (elapsed_ms >= BARCODE_PRECHECK_MS) {
                    if (segs > 0) {
                        printf("[BC_PRECHK] Timeout with %lu edges (need %d)\n", 
                               (unsigned long)segs, BARCODE_MIN_PRECHECK_SEGMENTS);
                    } else {
                        printf("[BC_PRECHK] No digital pulses detected!\n");
                        printf("   Check: GP7 connection, sensor power, digital output\n");
                    }
                    barcode_capture_abort();
                    stop_now();
                    barcode_set_cooldown_ms(BARCODE_FALSE_TRIGGER_COOLDOWN_MS);
                    prevL = prevR = 0;
                    st = ST_FOLLOW;
                    goto next_iter;
                }
                
                // Early exit if returned to white
                if (barcode_is_white()) {
                    printf("[BC_PRECHK] Returned to white, false trigger\n");
                    barcode_capture_abort();
                    stop_now();
                    barcode_set_cooldown_ms(BARCODE_FALSE_TRIGGER_COOLDOWN_MS);
                    prevL = prevR = 0;
                    st = ST_FOLLOW;
                    goto next_iter;
                }
            } break;

            case ST_BC_TRIGGERED: {
                printf("[BC_TRIG] Confirmed barcode, stopping\n");
                stop_now();
                sleep_ms(100);
                st = ST_BC_POSITION;
                t_state_start = get_absolute_time();
            } break;

            case ST_BC_POSITION: {
                printf("[BC_POS] Backing up %dms at PWM=%d\n", 
                       BARCODE_BACKUP_MS, BARCODE_BACKUP_PWM);
                drive_backward(BARCODE_BACKUP_PWM);
                sleep_ms(BARCODE_BACKUP_MS);
                stop_now();
                sleep_ms(100);

                printf("[BC_POS] Starting capture crawl at PWM=%d\n", BARCODE_CRAWL_PWM);
                barcode_capture_start();

                st = ST_BC_SCAN;
                t_state_start = get_absolute_time();
            } break;

            case ST_BC_SCAN: {
                drive_straight(BARCODE_CRAWL_PWM);

                bool done = barcode_capture_update();
                if (done) {
                    uint32_t elapsed = (uint32_t)(absolute_time_diff_us(t_state_start, 
                                                  get_absolute_time())/1000);
                    uint32_t segs = barcode_capture_segment_count();
                    printf("[BC_SCAN] Capture complete: %lu segments in %lums\n", 
                           (unsigned long)segs, (unsigned long)elapsed);
                    stop_now();
                    sleep_ms(50);
                    st = ST_BC_DECODE;
                }
            } break;

            case ST_BC_DECODE: {
                char c = barcode_decode_last();
                
                if (c >= 'A' && c <= 'Z') {
                    bool goRight = is_turn_right_letter(c);
                    last_dir = goRight ? +1 : -1;
                    printf("\n=================================\n");
                    printf("  BARCODE SUCCESS: '%c'\n", c);
                    printf("  Direction: %s\n", goRight ? "RIGHT" : "LEFT");
                    printf("=================================\n\n");
                    led_blink(4, 60);

                    // Turn and reacquire
                    printf("Turning and reacquiring line...\n");
                    absolute_time_t t0 = get_absolute_time();
                    bool found = false;
                    
                    while (absolute_time_diff_us(t0, get_absolute_time()) < 
                           (int64_t)REACQUIRE_MAX_MS*1000) {
                        if (last_dir < 0) 
                            swing_left(SPEED_REACQUIRE_PWM, SPEED_TURN_SLOW_PWM);
                        else              
                            swing_right(SPEED_REACQUIRE_PWM, SPEED_TURN_SLOW_PWM);

                        uint16_t r2 = read_adc_channel(LINE_ADC_CHANNEL, 4);
                        float ema2 = (1.0f - EMA_ALPHA) * ema + EMA_ALPHA * (float)r2;
                        bool white2 = ir_detect_surface((uint16_t)ema2, true, &g_cal);
                        
                        if (!white2) {
                            printf("Line reacquired!\n");
                            sleep_ms(REACQUIRE_HOLD_MS);
                            barcode_set_cooldown_ms(BC_COOLDOWN_MS);
                            t_last_nonwhite = get_absolute_time();
                            prevL = prevR = 0;
                            found = true;
                            st = ST_FOLLOW;
                            goto next_iter;
                        }
                        sleep_ms(LOOP_DT_MS);
                    }
                    
                    if (!found) {
                        printf("Reacquire timeout, starting scan\n");
                        t_state_start = get_absolute_time();
                        scan_min_left=scan_min_right=0xFFFF;
                        prevL = prevR = 0;
                        st = ST_LOST_WAIT;
                    }
                } else {
                    printf("\n=================================\n");
                    printf("  BARCODE DECODE FAILED\n");
                    printf("  Character: '%c'\n", c);
                    printf("=================================\n\n");
                    led_blink(2, 200);
                    
                    barcode_set_cooldown_ms(BC_COOLDOWN_MS);
                    t_last_nonwhite = get_absolute_time();
                    prevL = prevR = 0;
                    st = ST_FOLLOW;
                }
            } break;

            case ST_LOST_WAIT: {
                if (absolute_time_diff_us(t_state_start, get_absolute_time()) >= 25000) {
                    t_state_start = get_absolute_time();
                    st = ST_SCAN_LEFT;
                }
                if (!is_white) st = ST_FOLLOW;
                stop_now();
            } break;

            case ST_SCAN_LEFT: {
                swing_left(SPEED_TURN_FAST_PWM, SPEED_TURN_SLOW_PWM);
                if (filt < scan_min_left) scan_min_left = filt;
                if (!is_white) {
                    left_hold += LOOP_DT_MS;
                    if (left_hold >= BLACK_CONFIRM_MS) { 
                        last_dir = -1; 
                        st = ST_TURN_REACQUIRE; 
                    }
                } else left_hold = 0;

                if (absolute_time_diff_us(t_state_start, get_absolute_time()) >= 
                    (int64_t)SCAN_SWEEP_MS*1000) {
                    t_state_start = get_absolute_time();
                    st = ST_SCAN_RECENTER;
                }
            } break;

            case ST_SCAN_RECENTER: {
                swing_right(SPEED_TURN_FAST_PWM, SPEED_TURN_SLOW_PWM);
                if (absolute_time_diff_us(t_state_start, get_absolute_time()) >= 
                    (int64_t)SCAN_RECENTER_MS*1000) {
                    t_state_start = get_absolute_time();
                    st = ST_SCAN_RIGHT;
                }
            } break;

            case ST_SCAN_RIGHT: {
                swing_right(SPEED_TURN_FAST_PWM, SPEED_TURN_SLOW_PWM);
                if (filt < scan_min_right) scan_min_right = filt;
                if (!is_white) {
                    right_hold += LOOP_DT_MS;
                    if (right_hold >= BLACK_CONFIRM_MS) { 
                        last_dir = +1; 
                        st = ST_TURN_REACQUIRE; 
                    }
                } else right_hold = 0;

                if (absolute_time_diff_us(t_state_start, get_absolute_time()) >= 
                    (int64_t)SCAN_SWEEP_MS*1000) {
                    last_dir = (scan_min_left < scan_min_right) ? -1 : +1;
                    st = ST_TURN_REACQUIRE;
                }
            } break;

            case ST_TURN_REACQUIRE: {
                if (last_dir < 0) 
                    swing_left(SPEED_REACQUIRE_PWM, SPEED_TURN_SLOW_PWM);
                else              
                    swing_right(SPEED_REACQUIRE_PWM, SPEED_TURN_SLOW_PWM);

                if (!is_white) {
                    sleep_ms(REACQUIRE_HOLD_MS);
                    t_last_nonwhite = get_absolute_time();
                    prevL = prevR = 0;
                    st = ST_FOLLOW;
                } else {
                    if (absolute_time_diff_us(t_state_start, get_absolute_time()) >= 
                        (int64_t)REACQUIRE_MAX_MS*1000) {
                        last_dir = -last_dir;
                        t_state_start = get_absolute_time();
                    }
                }
            } break;
        }

    next_iter:
        ;
    }

    return 0;
}