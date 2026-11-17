#pragma once
// =============================================================================
// config.h — Compile-time defaults and hard firmware limits
//
// Single source of truth for:
//   * DEFAULT pin assignments (pins are remappable at runtime from the UI,
//     Settings → Hardware, and persisted in NVS; these are the factory values
//     used on first boot or after "reset hardware to defaults")
//   * hard safety ceilings that no control mode may exceed
//   * timing constants
//
// Deliberately free of Arduino includes so the pure-logic modules (and the
// native unit tests) can use it.
// =============================================================================

#include <cstdint>

// ---- Channels ---------------------------------------------------------------
// Two pump channels today. Raising this is mostly a matter of adding default
// pins below; each channel gets its own I²C controller (the ESP32 has two), so
// >2 flow sensors needs a TCA9548A mux (see README).
constexpr int MAX_CHANNELS = 2;

// ---- Default pin map (ESP32-WROOM-32 DevKit V1, 30-pin) ---------------------
// Strapping (0,2,5,12,15), flash (6-11), UART0 (1,3) and input-only (34-39)
// pins are avoided for outputs; config/pin_rules.cpp enforces this.
// -1 = not connected.
struct DefaultChannelPins {
    int uln[4];        // ULN2003 IN1..IN4
    int stepDirEn[3];  // STEP, DIR, EN
    int dcIn[2];       // DRV8871 IN1, IN2  (or PWM, DIR for generic H-bridge)
    int sda, scl;      // flow sensor I²C bus for this channel
};
constexpr DefaultChannelPins DEFAULT_PINS[MAX_CHANNELS] = {
    { {16, 17, 18, 19}, {16, 17, 18}, {16, 17}, 21, 22 },   // Ch 1 → I²C bus 0
    { {13, 14, 27, 26}, {13, 14, 27}, {13, 14}, 32, 33 },   // Ch 2 → I²C bus 1
};

// Status LED (DevKit V1 blue LED is on GPIO2, a strapping pin — output only
// after boot, which is fine for an LED).
constexpr int PIN_STATUS_LED = 2;

// ---- Hard safety limits -----------------------------------------------------
// Firmware-level ceilings, independent of any "smart" control logic. Every
// flow-setpoint mode (closed-loop, volume-over-time, program steps, open-loop
// flow) REJECTS setpoints outside [MIN, MAX] rather than clamping them, and a
// measured flow above MAX × FLOW_CEILING_TRIP_FACTOR for longer than
// FLOW_CEILING_TRIP_S trips the channel to FAULT in every mode (including
// manual and calibration).
constexpr float MAX_FLOW_RATE_ML_HR = 50.0f;    // = 833 µL/min
constexpr float MIN_FLOW_RATE_ML_HR = 0.0f;
constexpr float FLOW_CEILING_TRIP_FACTOR = 1.10f;
constexpr float FLOW_CEILING_TRIP_S = 5.0f;

constexpr float ML_HR_TO_UL_MIN = 1000.0f / 60.0f;
constexpr float MAX_FLOW_UL_MIN = MAX_FLOW_RATE_ML_HR * ML_HR_TO_UL_MIN;
constexpr float MIN_FLOW_UL_MIN = MIN_FLOW_RATE_ML_HR * ML_HR_TO_UL_MIN;

// Absolute driver-level command ceilings (the per-profile maxCommand must be
// at or below these; the motor driver clamps to the profile value).
constexpr float ABS_MAX_STEPPER_RPM = 600.0f;
constexpr float ABS_MAX_STEP_RATE_HZ = 20000.0f;   // step generator limit
constexpr float ABS_MAX_DUTY_PCT = 100.0f;

// ---- Timing -----------------------------------------------------------------
constexpr uint32_t CONTROL_LOOP_PERIOD_MS = 10;    // real-time task tick (100 Hz)
constexpr uint32_t SENSOR_POLL_PERIOD_MS = 20;     // SLF3S poll (50 Hz)
constexpr uint32_t HISTORY_PERIOD_MS = 1000;       // history ring buffer sample rate
constexpr uint32_t HISTORY_RAM_BUDGET_BYTES = 64000;

// ---- Defaults for user settings ---------------------------------------------
constexpr float DEFAULT_WATCHDOG_TIMEOUT_S = 10.0f;
constexpr float DEFAULT_TELEMETRY_HZ = 5.0f;
constexpr const char* DEFAULT_AP_PASSWORD = "biopump123";   // ≥ 8 chars (WPA2)
constexpr const char* DEFAULT_HOSTNAME = "biopump";
