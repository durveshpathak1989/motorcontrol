# Test Quad MotorControl Library

This library owns ESC PWM setup and motor output writes for the X quad layout.

## Pin Map

| Motor | ESP32 pin | Notes |
| --- | ---: | --- |
| Front left (FL) | GPIO 25 | ESC signal |
| Front right (FR) | GPIO 15 | ESC signal |
| Rear left (RL) | GPIO 14 | ESC signal |
| Rear right (RR) | GPIO 32 | ESC signal |

Connect ESC grounds to ESP32 ground. Keep propellers removed when testing.

## Main INO Integration Example

```cpp
#include "MotorControl.h"

void setup() {
    motorBegin();
    motorEscArm();     // Sends minimum throttle during ESC startup.
    motorOff();
}

static void writeMotors(float fl, float fr, float rl, float rr) {
    motorSet(fl, fr, rl, rr);
}

void loop() {
    float fl = 0.10f;
    float fr = 0.10f;
    float rl = 0.10f;
    float rr = 0.10f;
    writeMotors(fl, fr, rl, rr);
}
```


## Why These Data Types

Motor commands use `float` normalized throttle values because the controller mixes PID corrections before final PWM conversion. GPIO numbers are compile-time constants so the firmware has a single pin map and avoids accidental remapping during flight.
