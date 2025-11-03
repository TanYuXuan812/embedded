// functions.c - FIXED BARCODE IMPLEMENTATION
#include "functions.h"
#include <string.h>
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "pico/time.h"
#include <stdio.h>

// Forward declaration of ISR
static void barcode_edge_isr(uint gpio, uint32_t events);

// =================== General init ===================
void setup_dual_sensors(void) {
    stdio_init_all();

    // ADC init
    adc_init();
    adc_gpio_init(LINE_ADC_GPIO);
    adc_gpio_init(BARCODE_ADC_GPIO);

    // BARCODE digital pin setup
    gpio_init(BARCODE_DIGITAL_PIN);
    gpio_set_dir(BARCODE_DIGITAL_PIN, GPIO_IN);
    gpio_pull_up(BARCODE_DIGITAL_PIN);
    
    // Don't enable IRQ yet - will enable when ready to capture
    gpio_set_irq_enabled_with_callback(
        BARCODE_DIGITAL_PIN,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,  // disabled initially
        &barcode_edge_isr
    );
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
//                 BARCODE (Code 39)
// ===================================================

static uint16_t s_bc_thresh = 0;
static bool     s_bc_white_high = true;

bool barcode_is_white(void) {
    uint16_t raw = read_adc_channel(BARCODE_ADC_CHANNEL, 4);
    return s_bc_white_high ? (raw > s_bc_thresh) : (raw < s_bc_thresh);
}

// Trigger detection
static absolute_time_t s_bc_black_start = {0};
static absolute_time_t s_bc_cooldown_until = {0};

bool barcode_in_cooldown(void) {
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
    printf("[BARCODE_INIT] Threshold: %u, white_high: %d\n", analog_threshold, white_is_higher);
}

// -------------- Digital pulse capture (ISR) --------------
static volatile bool  cap_active = false;
static volatile bool  cap_done   = false;
static volatile uint32_t seg_count = 0;
static volatile uint32_t seg_us[MAX_BARCODE_SEGMENTS];  // Changed to microseconds for better precision
static volatile absolute_time_t last_edge_time;
static volatile bool last_level = false;
static volatile absolute_time_t cap_start_time;

static void barcode_edge_isr(uint gpio, uint32_t events) {
    if (!cap_active) return;
    
    absolute_time_t now = get_absolute_time();
    bool level = gpio_get(BARCODE_DIGITAL_PIN);

    // First edge - just record it
    if (to_us_since_boot(last_edge_time) == 0) {
        last_edge_time = now;
        last_level = level;
        return;
    }

    // Calculate duration in microseconds for better precision
    int64_t dur_us = absolute_time_diff_us(last_edge_time, now);
    
    // Ignore glitches (< 100us)
    if (dur_us < 100) {
        return;
    }
    
    last_edge_time = now;

    // Store pulse duration
    if (seg_count < MAX_BARCODE_SEGMENTS) {
        seg_us[seg_count++] = (uint32_t)dur_us;
    } else {
        // Overflow
        cap_done = true;
        cap_active = false;
    }

    last_level = level;
}

void barcode_capture_start(void) {
    
    
    // Clear state
    memset((void*)seg_us, 0, sizeof(seg_us));
    seg_count = 0;
    cap_done = false;
    last_edge_time = (absolute_time_t){0};
    cap_start_time = get_absolute_time();
    
    // Clear any pending interrupts
    gpio_acknowledge_irq(BARCODE_DIGITAL_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
    gpio_set_irq_enabled(BARCODE_DIGITAL_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    // Enable capture
    cap_active = true;
    
    // Enable IRQ
    gpio_set_irq_enabled(BARCODE_DIGITAL_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    
    printf("[CAPTURE_START] Armed and ready\n");
}

bool barcode_capture_update(void) {
    if (!cap_active && cap_done) return true;

    // Timeout check
    int64_t elapsed_us = absolute_time_diff_us(cap_start_time, get_absolute_time());
    if (elapsed_us >= (int64_t)BARCODE_CAPTURE_TIMEOUT_MS * 1000) {
        cap_active = false;
        cap_done = true;
        gpio_set_irq_enabled(BARCODE_DIGITAL_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
        printf("[CAPTURE] Timeout after %lld ms\n", elapsed_us / 1000);
        return true;
    }

    // End condition: sufficient segments + stable white
    if (seg_count >= 18) {  // Code39 needs at least 18 edges (9 bars + 9 spaces)
        if (to_us_since_boot(last_edge_time) != 0) {
            int64_t idle_us = absolute_time_diff_us(last_edge_time, get_absolute_time());
            uint32_t idle_ms = (uint32_t)(idle_us / 1000);
            
            if (idle_ms >= BARCODE_END_WHITE_MS && barcode_is_white()) {
                cap_active = false;
                cap_done = true;
                gpio_set_irq_enabled(BARCODE_DIGITAL_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
                printf("[CAPTURE] Complete: %lu segments, %lu ms idle\n", 
                       (unsigned long)seg_count, (unsigned long)idle_ms);
                return true;
            }
        }
    }

    return false;
}

void barcode_print_capture_debug(void) {
    printf("\n=== BARCODE CAPTURE DEBUG ===\n");
    printf("Segments captured: %lu\n", (unsigned long)seg_count);
    
    if (seg_count == 0) {
        printf("NO PULSES CAPTURED!\n");
        printf("Troubleshooting:\n");
        printf("  1. Check GP7 connection to barcode sensor DOUT\n");
        printf("  2. Verify barcode sensor has power (3.3V/5V and GND)\n");
        printf("  3. Check sensor is outputting digital signal (use oscilloscope/logic analyzer)\n");
        // printf("  4. Try slower speed (current: %d PWM)\n", BARCODE_CRAWL_PWM);
        printf("  5. Ensure barcode is high contrast and clean\n");
        printf("  6. Verify pull-up resistor is working (should read HIGH when idle)\n");
        uint16_t bc_analog = read_adc_channel(BARCODE_ADC_CHANNEL, 10);
        bool digital = gpio_get(BARCODE_DIGITAL_PIN);
        printf("  Current readings: Analog=%u, Digital=%s\n", 
               bc_analog, digital ? "HIGH" : "LOW");
        printf("=============================\n\n");
        return;
    }
    
    printf("Pulse durations (ms):\n");
    for (uint32_t i = 0; i < seg_count && i < 32; i++) {
        float ms = seg_us[i] / 1000.0f;
        printf("  [%2lu] %6.2fms", (unsigned long)i, ms);
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
        sum += seg_us[i];
        if (seg_us[i] < min_dur) min_dur = seg_us[i];
        if (seg_us[i] > max_dur) max_dur = seg_us[i];
    }
    float avg_ms = seg_count > 0 ? (sum / (float)seg_count) / 1000.0f : 0;
    
    printf("\nStats: min=%.2fms, max=%.2fms, avg=%.2fms\n", 
           min_dur / 1000.0f, max_dur / 1000.0f, avg_ms);
    printf("Ratio: max/min = %.2f (Code39 expects ~2.5-3.0)\n", 
           min_dur > 0 ? (float)max_dur / (float)min_dur : 0.0f);
    printf("=============================\n\n");
}

// ======= Code 39 robust decoder (bars+spaces, 9 elements per symbol) =======
typedef struct { char ch; const char *pat; int val; } c39_entry;
static const c39_entry C39[] = {
  {'0',"nnnwwnwnn", 0}, {'1',"wnnwnnnnw", 1}, {'2',"nnwwnnnnw", 2}, {'3',"wnwwnnnnn", 3},
  {'4',"nnnwwnnnw", 4}, {'5',"wnnwwnnnn", 5}, {'6',"nnwwwnnnn", 6}, {'7',"nnnwnnwnw", 7},
  {'8',"wnnwnnwnn", 8}, {'9',"nnwwnnwnn", 9},
  {'A',"wnnnnwnnw",10}, {'B',"nnwnnwnnw",11}, {'C',"wwnnnwnnn",12}, {'D',"nnnnwwnnw",13},
  {'E',"wnnnwwnnn",14}, {'F',"nnwnwwnnn",15}, {'G',"nnnnnwwnw",16}, {'H',"wnnnnwwnn",17},
  {'I',"nnwnnwwnn",18}, {'J',"nnnnwwwnn",19},
  {'K',"wnnnnnnww",20}, {'L',"nnwnnnnww",21}, {'M',"wnwnnnnwn",22}, {'N',"nnnnwnnww",23},
  {'O',"wnnnwnnwn",24}, {'P',"nnwnwnnwn",25}, {'Q',"nnnnnnwww",26}, {'R',"wnnnnnwwn",27},
  {'S',"nnwnnnwwn",28}, {'T',"nnnnwnwwn",29},
  {'U',"wwnnnnnnw",30}, {'V',"nwwnnnnnw",31}, {'W',"wwwnnnnnn",32}, {'X',"nwnnwnnnw",33},
  {'Y',"wwnnwnnnn",34}, {'Z',"nwwnwnnnn",35},
  {'-',"nwnnnnwnw",36}, {'.',"wwnnnnwnn",37}, {' ',"nwwnnnwnn",38},
  {'$',"nwnwnwnnn",39}, {'/',"nwnwnnnwn",40}, {'+', "nwnnnwnwn",41}, {'%',"nnnwnwnwn",42},
  {'*',"nwnnwnwnn",-1}
};
#define WIDE_NUM 25u  // 2.5× cutoff
#define WIDE_DEN 10u

static inline bool c39_is_wide(uint32_t w, uint32_t sum9){
  return (w * 15u * WIDE_DEN) >= (WIDE_NUM * sum9); // unit = sum9/15
}

static uint16_t widths_to_mask(const uint16_t *w9, int *wide_ct){
  uint32_t sum9=0; for(int i=0;i<9;i++) sum9+=w9[i];
  uint16_t m=0; int wc=0;
  for(int i=0;i<9;i++){
    bool wide=c39_is_wide(w9[i],sum9);
    m=(uint16_t)((m<<1)|(wide?1:0)); wc+=wide;
  }
  if(wide_ct) *wide_ct=wc; return m;
}

static bool mask_eq_pat(uint16_t mask, const char *pat){
  for(int i=0;i<9;i++){
    bool want=(pat[i]=='w'||pat[i]=='W');
    bool got = (mask & (1u<<(8-i)))!=0;
    if(want!=got) return false;
  } return true;
}

static int pat_to_char(uint16_t mask, char *out_char, int *out_val){
  for(size_t i=0;i<sizeof(C39)/sizeof(C39[0]);i++){
    if(mask_eq_pat(mask, C39[i].pat)){
      if(out_char)*out_char=C39[i].ch;
      if(out_val)*out_val=C39[i].val;
      return 1;
    }
  }
  return 0;
}

static int find_star(const uint16_t *seg,int n,int bar_parity){
  for(int i=0;i+9<n;i++){
    if((i&1)!=bar_parity) continue;
    int wc=0; uint16_t mask=widths_to_mask(&seg[i],&wc);
    if(wc!=3) continue;
    char ch=0; if(!pat_to_char(mask,&ch,NULL)) continue;
    if(ch!='*') continue;
    // enforce narrow gap if present
    if(i+9<n){
      uint32_t sum9=0; for(int k=0;k<9;k++) sum9+=seg[i+k];
      if(c39_is_wide(seg[i+9],sum9)) continue;
    }
    return i;
  }
  return -1;
}

static int decode_forward(const uint16_t *seg,int n,int i,char *out,int out_max,bool mod43){
  int oi=0, sum=0; bool started=false;
  while(1){
    if(i+9>n) return -1;
    int wc=0; uint16_t mask=widths_to_mask(&seg[i],&wc);
    if(wc!=3) return -2;
    char ch=0; int val=-1;
    if(!pat_to_char(mask,&ch,&val)) return -3;
    i+=9; if(i<n) i+=1; // consume narrow gap if present
    if(ch=='*'){
      if(!started){ started=true; sum=0; continue; } // opening *
      // closing *
      if(mod43 && oi>0){
        // verify last is check
        char chk=out[oi-1]; int chkval=-1;
        for(size_t k=0;k<sizeof(C39)/sizeof(C39[0]);k++) if(C39[k].ch==chk){ chkval=C39[k].val; break; }
        if(chkval<0) return -4;
        if(((sum-chkval)%43)!=chkval) return -5;
        oi-=1;
      }
      out[ (oi<out_max?oi:out_max-1) ] = '\0';
      return oi;
    }else{
      if(!started) return -6;
      if(oi<out_max-1) out[oi++]=ch;
      if(val>=0) sum=(sum+val)%43;
    }
  }
}

// public: decode from a capture buffer of widths
static int code39_decode_segments(const uint16_t *seg,int n,char *out,int out_max,bool mod43){
  if(n<10) return -10;

  // forward
  for(int p=0;p<2;p++){
    int s=find_star(seg,n,p);
    if(s>=0){ int r=decode_forward(seg,n,s,out,out_max,mod43); if(r>=0) return r; }
  }
  // reversed
  static uint16_t rev[2048];
  if(n>(int)(sizeof(rev)/sizeof(rev[0]))) return -11;
  for(int i=0;i<n;i++) rev[i]=seg[n-1-i];
  for(int p=0;p<2;p++){
    int s=find_star(rev,n,p);
    if(s>=0){ int r=decode_forward(rev,n,s,out,out_max,mod43); if(r>=0) return r; }
  }
  return -12;
}
// Improved narrow/wide classification using K-means clustering
static int classify_narrow_wide(const uint32_t *dur_us, uint32_t n, uint8_t *out_bits) {
    if (n == 0) return 0;
    
    // Copy and sort for median calculation
    uint32_t tmp[MAX_BARCODE_SEGMENTS];
    uint32_t m = n > MAX_BARCODE_SEGMENTS ? MAX_BARCODE_SEGMENTS : n;
    for (uint32_t i = 0; i < m; i++) tmp[i] = dur_us[i];
    
    // Bubble sort
    for (uint32_t i = 0; i < m - 1; i++) {
        for (uint32_t j = 0; j < m - i - 1; j++) {
            if (tmp[j] > tmp[j + 1]) {
                uint32_t t = tmp[j];
                tmp[j] = tmp[j + 1];
                tmp[j + 1] = t;
            }
        }
    }
    
    // Calculate median
    uint32_t median_us = (m % 2 == 0) ? (tmp[m/2 - 1] + tmp[m/2]) / 2 : tmp[m/2];
    
    // Use 1.5x median as threshold (more aggressive than 1.8x)
    float threshold_us = median_us * 1.5f;
    
    printf("[DECODE] Median: %.2fms, Threshold: %.2fms\n", 
           median_us / 1000.0f, threshold_us / 1000.0f);
    printf("[DECODE] Pattern: ");
    
    for (uint32_t i = 0; i < n; i++) {
        out_bits[i] = (dur_us[i] > (uint32_t)threshold_us) ? 1 : 0;
        printf("%c", out_bits[i] ? 'W' : 'n');
    }
    printf("\n");

    return (int)n;
}

char barcode_decode_last(void) {
    // Debug dump of captured segments
    barcode_print_capture_debug();

    // Copy volatile capture (ms) -> uint16_t array the decoder expects
    uint32_t n32 = seg_count;
    if (n32 > MAX_BARCODE_SEGMENTS) n32 = MAX_BARCODE_SEGMENTS;
    if (n32 < 10) { printf("[BC_FAIL] too few segments (%lu)\n", (unsigned long)n32); return '?'; }

    uint16_t buf[2048];
    int n = (int)n32;
    for (int i=0;i<n;i++) {
        uint32_t v = seg_us[i];
        buf[i] = (v > 0xFFFFu) ? 0xFFFFu : (uint16_t)v;
    }

    char out[64];
    int res = code39_decode_segments(buf, n, out, sizeof(out), /*use_mod43=*/false);
    if (res >= 0) {
        printf("[BC_OK] \"%s\" (%d chars)\n", out, res);
        // Return first letter (keeps your A/B/C→direction rule in main.c)
        for (int i=0;i<res;i++) if ((out[i]>='A'&&out[i]<='Z')) return out[i];
        return out[0]; // fallback to first symbol (could be digit/punct/space)
    } else {
        printf("[BC_FAIL] decode err %d (seg_count=%d)\n", res, n);
        return '?';
    }
}

void barcode_capture_abort(void) {
    cap_active = false;
    cap_done = false;
    seg_count = 0;
    last_edge_time = (absolute_time_t){0};
    gpio_set_irq_enabled(BARCODE_DIGITAL_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
    printf("[CAPTURE] Aborted\n");
}

uint32_t barcode_capture_segment_count(void) {
    return seg_count;
}

bool barcode_capture_is_active(void) {
    return cap_active;
}