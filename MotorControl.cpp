/**
 * ============================================================
 *  MotorControl.cpp — Fast non-blocking ESC PWM motor output
 *  ESP32 / HUZZAH32 · X-frame Quadcopter
 * ============================================================
 *
 *  Fix rationale:
 *    Your timing log showed phase motor≈39800us immediately after arming.
 *    That means the blocking was in motorSet()/motorOff(), not IMU/PID.
 *
 *    This version avoids pin-keyed LEDC auto-channel writes and uses explicit
 *    LEDC channels. On Arduino-ESP32 core 3.x it uses ledcAttachChannel() and
 *    ledcWriteChannel(). On core 2.x it uses ledcSetup()/ledcAttachPin() and
 *    ledcWrite(channel,duty).
 *
 *    Default output is 400 Hz standard PWM with 1000–2000 us pulse width.
 *    If your ESCs only support 50 Hz, change MOTOR_PWM_FREQ_HZ in the header.
 * ============================================================
 */

#include "MotorControl.h"
#include "DebugConfig.h"

#if __has_include(<esp_arduino_version.h>)
  #include <esp_arduino_version.h>
#endif

// ─────────────────────────────────────────────────────────────
//  Internal state
// ─────────────────────────────────────────────────────────────
static MotorState_t s_state = {0.0f, 0.0f, 0.0f, 0.0f, false};

static const uint8_t s_pins[4] = {
    MOTOR_PIN_FL, MOTOR_PIN_FR, MOTOR_PIN_RL, MOTOR_PIN_RR
};

static const uint8_t s_channels[4] = {
    MOTOR_CH_FL, MOTOR_CH_FR, MOTOR_CH_RL, MOTOR_CH_RR
};

// Last pulse written to each channel. Avoids redundant register writes when
// the command is unchanged, especially when armed but throttle is still zero.
static uint16_t s_lastUs[4] = {0, 0, 0, 0};

// ─────────────────────────────────────────────────────────────
//  Private helpers
// ─────────────────────────────────────────────────────────────
static float _clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static uint32_t _maxDuty(void) {
    return (1UL << MOTOR_PWM_RESOLUTION) - 1UL;
}

/** Convert pulse width in µs to LEDC duty counts for the configured PWM freq. */
static uint32_t _usToDuty(uint16_t us) {
    const uint32_t periodUs = 1000000UL / MOTOR_PWM_FREQ_HZ;
    const uint32_t maxDuty  = _maxDuty();
    if (us > periodUs) us = periodUs;
    return ((uint32_t)us * maxDuty) / periodUs;
}

static void _writeDuty(uint8_t ch, uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWriteChannel(ch, duty);
#else
    ledcWrite(ch, duty);
#endif
}

/** Write a specific pulse width to one motor channel. */
static void _writeUsIdx(uint8_t idx, uint16_t us, bool force = false) {
    if (idx >= 4) return;
    if (!force && s_lastUs[idx] == us) return;
    s_lastUs[idx] = us;
    _writeDuty(s_channels[idx], _usToDuty(us));
}

static void _allUs(uint16_t us, bool force = false) {
    for (int i = 0; i < 4; i++) _writeUsIdx(i, us, force);
}

static void _attachChannels(void) {
    for (int i = 0; i < 4; i++) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
        // Core 3.x explicit channel API.
        ledcAttachChannel(s_pins[i], MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION, s_channels[i]);
#else
        // Core 2.x channel API.
        ledcSetup(s_channels[i], MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION);
        ledcAttachPin(s_pins[i], s_channels[i]);
#endif
        s_lastUs[i] = 0;
    }
}

// ─────────────────────────────────────────────────────────────
//  Public API implementation
// ─────────────────────────────────────────────────────────────
void motorBegin(void) {
    DBG_PRINTLN(F("[MOTOR] Initialising fast PWM channels..."));

    pinMode(27, INPUT);  // MPU INT safety: never drive it as output

    _attachChannels();
    _allUs(MOTOR_US_MIN, true);
    s_state = {0.0f, 0.0f, 0.0f, 0.0f, false};

    DBG_PRINTF("[MOTOR] FL=GPIO%d/ch%d  FR=GPIO%d/ch%d  RL=GPIO%d/ch%d  RR=GPIO%d/ch%d\n",
                  MOTOR_PIN_FL, MOTOR_CH_FL,
                  MOTOR_PIN_FR, MOTOR_CH_FR,
                  MOTOR_PIN_RL, MOTOR_CH_RL,
                  MOTOR_PIN_RR, MOTOR_CH_RR);
    DBG_PRINTF("[MOTOR] PWM: %d Hz  %d-bit  %u–%u us\n",
                  MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION, MOTOR_US_MIN, MOTOR_US_MAX);
    DBG_PRINTLN(F("[MOTOR] All motors at minimum. Ready."));
}

bool motorArm(void) {
    DBG_PRINTLN(F("[MOTOR] Arming sequence..."));
    _allUs(MOTOR_US_ARM, true);
    delay(500);
    _allUs(MOTOR_US_MIN, true);
    delay(2000);
    s_state.armed = true;
    DBG_PRINTLN(F("[MOTOR] Armed. ESC ready."));
    return true;
}

void motorDisarm(void) {
    _allUs(MOTOR_US_ARM, true);
    s_state.armed  = false;
    s_state.fl = s_state.fr = s_state.rl = s_state.rr = 0.0f;
    DBG_PRINTLN(F("[MOTOR] Disarmed."));
}

void motorOff(void) {
    _allUs(MOTOR_US_ARM);
    s_state.armed  = false;
    s_state.fl = s_state.fr = s_state.rl = s_state.rr = 0.0f;
}

uint16_t motorUsFromThrottle(float t) {
    t = _clampf(t, 0.0f, 1.0f);
    return (uint16_t)(MOTOR_US_MIN + t * (float)(MOTOR_US_MAX - MOTOR_US_MIN));
}

void motorSet(float fl, float fr, float rl, float rr) {
    fl = _clampf(fl, MOTOR_THROTTLE_MIN, MOTOR_THROTTLE_MAX);
    fr = _clampf(fr, MOTOR_THROTTLE_MIN, MOTOR_THROTTLE_MAX);
    rl = _clampf(rl, MOTOR_THROTTLE_MIN, MOTOR_THROTTLE_MAX);
    rr = _clampf(rr, MOTOR_THROTTLE_MIN, MOTOR_THROTTLE_MAX);

    _writeUsIdx(0, motorUsFromThrottle(fl));
    _writeUsIdx(1, motorUsFromThrottle(fr));
    _writeUsIdx(2, motorUsFromThrottle(rl));
    _writeUsIdx(3, motorUsFromThrottle(rr));

    s_state.fl = fl; s_state.fr = fr;
    s_state.rl = rl; s_state.rr = rr;
    s_state.armed = true;
}

void motorSetAll(float throttle) {
    motorSet(throttle, throttle, throttle, throttle);
}

void motorMixerX(float throttle, float roll, float pitch, float yaw) {
    if (!s_state.armed) return;

    throttle = _clampf(throttle, 0.0f, MOTOR_THROTTLE_MAX);
    roll     = _clampf(roll,    -1.0f, 1.0f);
    pitch    = _clampf(pitch,   -1.0f, 1.0f);
    yaw      = _clampf(yaw,     -1.0f, 1.0f);

    const float AUTH = 0.25f;
    float r = roll  * AUTH;
    float p = pitch * AUTH;
    float y = yaw   * AUTH;

    float fl = throttle + p + r - y;
    float fr = throttle - p - r - y;
    float rl = throttle - p + r + y;
    float rr = throttle + p - r + y;

    float hi = max(max(fl, fr), max(rl, rr));
    if (hi > MOTOR_THROTTLE_MAX) {
        float scale = MOTOR_THROTTLE_MAX / hi;
        fl *= scale; fr *= scale; rl *= scale; rr *= scale;
    }

    float idle = (throttle > MOTOR_IDLE_THROTTLE) ? MOTOR_IDLE_THROTTLE : 0.0f;
    fl = _clampf(fl, idle, MOTOR_THROTTLE_MAX);
    fr = _clampf(fr, idle, MOTOR_THROTTLE_MAX);
    rl = _clampf(rl, idle, MOTOR_THROTTLE_MAX);
    rr = _clampf(rr, idle, MOTOR_THROTTLE_MAX);

    motorSet(fl, fr, rl, rr);
}

MotorState_t motorGetState(void) {
    return s_state;
}

void motorEscArm(void) {
    DBG_PRINTLN(F("[ESC] Sending arm pulse (1000 us x 3 s)..."));
    _allUs(MOTOR_US_MIN, true);
    delay(3000);
    DBG_PRINTLN(F("[ESC] ESC armed and ready."));
    s_state.armed = true;
}

void motorEscCalibrate(void) {
    _attachChannels();

    DBG_PRINTLN(F(""));
    DBG_PRINTLN(F("╔══════════════════════════════════════════════════════╗"));
    DBG_PRINTLN(F("║          ESC ENDPOINT CALIBRATION ROUTINE           ║"));
    DBG_PRINTLN(F("╠══════════════════════════════════════════════════════╣"));
    DBG_PRINTLN(F("║  REMOVE ALL PROPELLERS BEFORE CONTINUING            ║"));
    DBG_PRINTLN(F("╚══════════════════════════════════════════════════════╝"));
    DBG_PRINTLN(F("[CAL] Step 1 — Disconnect battery, then press any key."));
    while (!Serial.available()) delay(20);
    while (Serial.available()) Serial.read();

    DBG_PRINTLN(F("[CAL] Sending HIGH endpoint. Connect battery now."));
    _allUs(MOTOR_US_CAL_HIGH, true);
    delay(5000);

    DBG_PRINTLN(F("[CAL] Sending LOW endpoint."));
    _allUs(MOTOR_US_MIN, true);
    delay(5000);

    DBG_PRINTLN(F("[CAL] ESC calibration done. Power-cycle ESCs."));
    s_state.armed = false;
}
