//**
 /* ============================================================
 *  MotorControl.h — ESC PWM Motor Control
 *  Adafruit HUZZAH32 / ESP32 · X-frame Quadcopter
 * ============================================================
 */

#pragma once
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// ─────────────────────────────────────────────────────────────
//  Pin assignments — do NOT change unless rewiring the board
// ─────────────────────────────────────────────────────────────
#define MOTOR_PIN_FL   25    // Front-Left  CCW
#define MOTOR_PIN_FR   15    // Front-Right CW
#define MOTOR_PIN_RL   14    // Rear-Left   CW
#define MOTOR_PIN_RR   32    // Rear-Right  CCW

// ─────────────────────────────────────────────────────────────
//  ESC PWM parameters
// ─────────────────────────────────────────────────────────────
// IMPORTANT CHANGE:
// Use 400 Hz / 12-bit explicit LEDC channels instead of 50 Hz / 16-bit
// auto-attached pin writes. Your timing log showed motorSet() consuming
// ~39.8 ms when armed, which is consistent with blocking/glitchless updates
// against a 50 Hz PWM frame. Explicit channels + 400 Hz updates are intended
// to make motorSet() a fast register update.
//
// If your ESCs do not accept 400 Hz standard PWM, change this back to 50.
// The explicit-channel path should still be faster than the previous pin-keyed
// LEDC path, but 400 Hz is preferred for a 400 Hz flight loop.
#define MOTOR_PWM_FREQ_HZ      400
#define MOTOR_PWM_RESOLUTION   12      // 0–4095 counts; enough for 1000–2000 us at 400 Hz

#define MOTOR_US_MIN         1000      // µs → minimum throttle
#define MOTOR_US_MAX         2000      // µs → full throttle
#define MOTOR_US_ARM          900      // µs → below min, used to stop/disarm ESC
#define MOTOR_US_CAL_HIGH    2100      // µs → above max, used in calibration
#define MOTOR_US_IDLE        1050      // µs → just above min

// Explicit LEDC channels. Do not overlap with other LEDC users.
#define MOTOR_CH_FL   0
#define MOTOR_CH_FR   1
#define MOTOR_CH_RL   2
#define MOTOR_CH_RR   3

// ─────────────────────────────────────────────────────────────
//  Safety limits
// ─────────────────────────────────────────────────────────────
#define MOTOR_THROTTLE_MAX     0.95f
#define MOTOR_THROTTLE_MIN     0.0f
#define MOTOR_IDLE_THROTTLE    0.05f

// ─────────────────────────────────────────────────────────────
//  Motor state
// ─────────────────────────────────────────────────────────────
typedef struct {
    float fl;
    float fr;
    float rl;
    float rr;
    bool  armed;
} MotorState_t;

void motorBegin(void);
bool motorArm(void);
void motorDisarm(void);
void motorSet(float fl, float fr, float rl, float rr);
void motorSetAll(float throttle);
void motorOff(void);
void motorMixerX(float throttle, float roll, float pitch, float yaw);
MotorState_t motorGetState(void);
void motorEscCalibrate(void);
void motorEscArm(void);
uint16_t motorUsFromThrottle(float t);

#endif // MOTOR_CONTROL_H
