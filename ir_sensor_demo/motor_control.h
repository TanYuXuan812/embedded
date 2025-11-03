#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "pico/stdlib.h"

// Motor selection
#define MOTOR_LEFT  1
#define MOTOR_RIGHT 2

// Motor direction
#define FORWARD  1
#define BACKWARD 2

// PWM pins for ROBO PICO
#define M1A 10  // Left motor
#define M1B 11  // Left motor
#define M2A 8   // Right motor
#define M2B 9   // Right motor

// Encoder pins
#define ENCODER_LEFT 16
#define ENCODER_RIGHT 6

// PID structure
typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
} PID_Controller;

// Initialize motors (pins + PWM)
void setup_motor(void);

// Initialize encoders (interrupt-based period capture on rising edges)
void setup_encoders(void);

// Initialize PID controller
void init_pid(PID_Controller *pid, float kp, float ki, float kd);

// Compute PID output
float compute_pid(PID_Controller *pid, float set_point, float measured, float dt);

// Get measured speed for left motor (in Hz)
float get_left_speed(void);

// Get measured speed for right motor (in Hz)
float get_right_speed(void);

// Start a motor with direction and speed (0-65535)
void start_motor(int motor, int direction, uint16_t speed);

// Stop a motor
void stop_motor(int motor);

// Drive both motors together
void drive_both_motors(int direction, uint16_t speed);

// Get the latest captured period for left encoder (in microseconds)
uint64_t get_left_period(void);

// Get the latest captured period for right encoder (in microseconds)
uint64_t get_right_period(void);

#endif