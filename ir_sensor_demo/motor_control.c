#include "motor_control.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"

// Volatile variables for periods (updated in IRQ)
static volatile uint64_t left_period = 0;
static volatile uint64_t right_period = 0;

// Non-volatile last rise times (accessed only within IRQ context for each)
static uint64_t left_last_rise = 0;
static uint64_t right_last_rise = 0;

// IRQ callback for encoders (now only on rising edges for period measurement)
static void encoder_callback(uint gpio, uint32_t events) {
    if (events & GPIO_IRQ_EDGE_RISE) {
        uint64_t current_time = time_us_64();

        if (gpio == ENCODER_LEFT) {
            if (left_last_rise != 0) {
                left_period = current_time - left_last_rise;
            }
            left_last_rise = current_time;
        } else if (gpio == ENCODER_RIGHT) {
            if (right_last_rise != 0) {
                right_period = current_time - right_last_rise;
            }
            right_last_rise = current_time;
        }
    }

    // Acknowledge the interrupt
    gpio_acknowledge_irq(gpio, events);
}

// Helper: Initialize PWM for a pin
static void init_pwm_pin(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, 65535); // full range
    pwm_config_set_clkdiv(&config, 1.0f);
    pwm_init(slice, &config, false);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(pin), 0);
    pwm_set_enabled(slice, true);
}

// Setup both motors
void setup_motor(void) {
    init_pwm_pin(M1A);
    init_pwm_pin(M1B);
    init_pwm_pin(M2A);
    init_pwm_pin(M2B);
}

// Setup encoders
void setup_encoders(void) {
    // Initialize pins as inputs with pull-ups (assuming open-collector encoders; adjust if needed)
    gpio_init(ENCODER_LEFT);
    gpio_set_dir(ENCODER_LEFT, GPIO_IN);
    gpio_pull_up(ENCODER_LEFT);

    gpio_init(ENCODER_RIGHT);
    gpio_set_dir(ENCODER_RIGHT, GPIO_IN);
    gpio_pull_up(ENCODER_RIGHT);

    // Set up IRQ with callback (only rising edges for period)
    gpio_set_irq_enabled_with_callback(ENCODER_LEFT, GPIO_IRQ_EDGE_RISE, true, &encoder_callback);
    gpio_set_irq_enabled(ENCODER_RIGHT, GPIO_IRQ_EDGE_RISE, true);
}

// Initialize PID
void init_pid(PID_Controller *pid, float kp, float ki, float kd) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

// Compute PID
float compute_pid(PID_Controller *pid, float set_point, float measured, float dt) {
    float error = set_point - measured;
    pid->integral += error * dt;
    // Add integral windup limit (prevent excessive buildup)
    if (pid->integral > 50.0f) pid->integral = 50.0f;
    if (pid->integral < -50.0f) pid->integral = -50.0f;
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;
    return pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
}

// Get left speed (frequency in Hz = 1e6 / period_us)
float get_left_speed(void) {
    uint64_t period = get_left_period();
    if (period == 0) return 0.0f;
    return 1000000.0f / (float)period;
}

// Get right speed (frequency in Hz = 1e6 / period_us)
float get_right_speed(void) {
    uint64_t period = get_right_period();
    if (period == 0) return 0.0f;
    return 1000000.0f / (float)period;
}

// Start a motor with direction and speed
void start_motor(int motor, int direction, uint16_t speed) {
    if (motor == MOTOR_LEFT) {
        if (direction == FORWARD) {
            pwm_set_chan_level(pwm_gpio_to_slice_num(M1A), pwm_gpio_to_channel(M1A), speed);
            pwm_set_chan_level(pwm_gpio_to_slice_num(M1B), pwm_gpio_to_channel(M1B), 0);
        } else { // BACKWARD
            pwm_set_chan_level(pwm_gpio_to_slice_num(M1A), pwm_gpio_to_channel(M1A), 0);
            pwm_set_chan_level(pwm_gpio_to_slice_num(M1B), pwm_gpio_to_channel(M1B), speed);
        }
    } else if (motor == MOTOR_RIGHT) {
        if (direction == FORWARD) {
            pwm_set_chan_level(pwm_gpio_to_slice_num(M2A), pwm_gpio_to_channel(M2A), 0);
            pwm_set_chan_level(pwm_gpio_to_slice_num(M2B), pwm_gpio_to_channel(M2B), speed);
        } else { // BACKWARD
            pwm_set_chan_level(pwm_gpio_to_slice_num(M2A), pwm_gpio_to_channel(M2A), speed);
            pwm_set_chan_level(pwm_gpio_to_slice_num(M2B), pwm_gpio_to_channel(M2B), 0);
        }
    }
}

// Stop a motor
void stop_motor(int motor) {
    if (motor == MOTOR_LEFT) {
        pwm_set_chan_level(pwm_gpio_to_slice_num(M1A), pwm_gpio_to_channel(M1A), 0);
        pwm_set_chan_level(pwm_gpio_to_slice_num(M1B), pwm_gpio_to_channel(M1B), 0);
    } else if (motor == MOTOR_RIGHT) {
        pwm_set_chan_level(pwm_gpio_to_slice_num(M2A), pwm_gpio_to_channel(M2A), 0);
        pwm_set_chan_level(pwm_gpio_to_slice_num(M2B), pwm_gpio_to_channel(M2B), 0);
    }
}

// Drive both motors together
void drive_both_motors(int direction, uint16_t speed) {
    start_motor(MOTOR_LEFT, direction, speed);
    start_motor(MOTOR_RIGHT, direction, speed);
}

// Getters for periods
uint64_t get_left_period(void) {
    return left_period;
}

uint64_t get_right_period(void) {
    return right_period;
}