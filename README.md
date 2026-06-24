# Motor Control Library

## Purpose

Provides fast ESP32 LEDC PWM output for four ESCs in an X-frame quadcopter, including arming, disarming, throttle conversion, and motor mixing helpers.

## Files

- `MotorControl.h/.cpp`: Pin/channel definitions, PWM setup, ESC pulses, and motor state.

## Quick Start

```cpp
#include "MotorControl.h"

void setup() {
    motorBegin();
    motorDisarm();
}

void loop() {
    if (armed) {
        motorSet(0.10f, 0.10f, 0.10f, 0.10f);
    } else {
        motorOff();
    }
}
```

## How It Fits Into The Flight Controller

This library lives under `Submodules/MotorControl` in the main `Test_Quad` firmware
and is built as an Arduino library by adding `Submodules/` to the Arduino
library search path. The main firmware includes it directly from
`RC_FlightController.ino` or from another support module.

The flight controller runs a 400 Hz control loop on ESP32, so this library
should avoid heap allocation, long blocking calls, and unbounded Serial output
inside flight-critical paths. Debug output should use `DebugConfig.h` macros
where available so `VERBOSE_ON=0` builds can compile prints out.

## Data Type Choices

- `float` throttle: Normalized 0.0-1.0 motor commands make mixer math independent of pulse-width details.
- `uint16_t` microseconds: ESC PWM pulse widths are naturally 1000-2000 us values that fit in 16 bits.
- `uint8_t` channels: LEDC channel identifiers are small integer hardware resources.
- `MotorState_t`: Telemetry and safety code can read the last commanded motor state without touching PWM internals.

## Usage Guidance

1. Initialize hardware-facing classes once during `setup()`.
2. Keep update/read calls deterministic when used from a FreeRTOS task.
3. Prefer explicit validity flags over sentinel numeric values.
4. Keep units visible in field names, such as `_dps`, `_g`, `_uT`, `_m`, or `_us`.
5. When adding telemetry fields, update both the packet struct and JSON serializer.

## Example Build Integration

```bash
arduino-cli compile \
  --fqbn esp32:esp32:esp32:UploadSpeed=921600,CPUFreq=240,FlashFreq=80,FlashMode=qio,FlashSize=4M,PartitionScheme=min_spiffs,DebugLevel=none,PSRAM=disabled,LoopCore=1,EventsCore=1,EraseFlash=none,JTAGAdapter=default,ZigbeeMode=default \
  --libraries ./Submodules \
  .
```

For quiet flight builds:

```bash
arduino-cli compile ... --build-property compiler.cpp.extra_flags=-DVERBOSE_ON=0
```


## Integration Notes

In the main flight-controller sketch, this library is included through Arduino's
library search path. When this folder is converted to a git submodule, keep the
folder name stable under `Submodules/` so includes such as `#include "..."`
continue to resolve.

Most examples below are intentionally small. On the real flight controller,
objects are usually constructed globally, initialized once from `setup()`, and
then called from FreeRTOS tasks at deterministic rates.

