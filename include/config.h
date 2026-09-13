#pragma once
// =============================================================================
// config.h — Hardware/pin configuration and tunable constants
//
// Keep ALL pin assignments and hardware-specific numbers here, not scattered
// through the codebase. When you change breadboard wiring, this is the only
// file that should need editing.
// =============================================================================

#include <Arduino.h>

// ---- Stepper driver pins (e.g. A4988 / DRV8825 / TMC2209 style STEP/DIR) ----
constexpr gpio_num_t PIN_STEP    = GPIO_NUM_26;
constexpr gpio_num_t PIN_DIR     = GPIO_NUM_27;
constexpr gpio_num_t PIN_ENABLE  = GPIO_NUM_25;   // active LOW on most drivers

// ---- Optional: end-stop / occlusion / bubble sensors (wire up as needed) ----
constexpr gpio_num_t PIN_LIMIT_SWITCH = GPIO_NUM_34;   // input only pin on classic ESP32

// ---- Status LED ----
constexpr gpio_num_t PIN_STATUS_LED = GPIO_NUM_2;

// ---- Mechanical / motion constants — FILL THESE IN FOR YOUR HARDWARE ----
// These convert "flow rate" (e.g. mL/hr) into step pulses. Placeholder values —
// replace with real numbers once you've picked a syringe/pump geometry and
// motor step angle + microstepping setting.
constexpr float STEPS_PER_REVOLUTION   = 200.0f;   // full steps/rev of the motor
constexpr float MICROSTEPPING          = 16.0f;    // microstepping factor set on the driver
constexpr float LEAD_SCREW_MM_PER_REV  = 1.0f;     // linear travel per rev (syringe pump)
constexpr float SYRINGE_AREA_MM2       = 50.0f;    // syringe barrel cross-sectional area (mL geometry)

// ---- Control loop timing ----
constexpr uint32_t CONTROL_LOOP_PERIOD_MS = 50;    // how often the flow control task runs

// ---- Safety limits — a real implantable device needs hard limits enforced
// in firmware, independent of whatever the "smart" control logic decides. ----
constexpr float MAX_FLOW_RATE_ML_HR = 50.0f;   // hard ceiling, reject anything above
constexpr float MIN_FLOW_RATE_ML_HR = 0.0f;
