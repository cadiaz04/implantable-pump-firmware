#pragma once
// =============================================================================
// motor_types.h — driver kinds, per-pump motor parameters and unit math.
// Pure (no Arduino) so it can be unit tested natively.
// =============================================================================
#include <cstdint>

enum class DriverType : uint8_t {
    ULN2003 = 0,    // unipolar stepper (28BYJ-48), 4-wire half-step
    STEP_DIR = 1,   // A4988 / DRV8825 / TMC2209 — bipolar NEMA steppers
    DRV8871 = 2,    // brushed DC, IN1/IN2 both PWM
    PWM_DIR = 3,    // brushed DC, generic PWM + DIR (L298N, TB6612)
    NONE = 255,
};

enum class DriverFamily : uint8_t { STEPPER, DC, NONE };

inline DriverFamily familyOf(DriverType t) {
    switch (t) {
        case DriverType::ULN2003:
        case DriverType::STEP_DIR: return DriverFamily::STEPPER;
        case DriverType::DRV8871:
        case DriverType::PWM_DIR: return DriverFamily::DC;
        default: return DriverFamily::NONE;
    }
}

const char* driverTypeId(DriverType t);              // "uln2003", "stepdir", ...
DriverType driverTypeFromId(const char* id);         // NONE if unknown
const char* commandUnitFor(DriverType t);            // "RPM" or "Duty %"
int requiredPinCount(DriverType t);                  // mandatory pins
int maxPinCount(DriverType t);                       // incl. optional EN

enum class DecayMode : uint8_t { FAST = 0, SLOW = 1 };

// Per-pump motor parameters (part of a PumpProfile).
struct MotorParams {
    DriverType driverType = DriverType::ULN2003;
    float stepsPerRev = 4096.0f;    // steps per MOTOR rev (28BYJ-48: 4096 half-steps incl. gearbox)
    float microstep = 1.0f;         // STEP/DIR microstep factor (1 for ULN2003)
    float gearRatio = 1.0f;         // motor revs per driven-shaft rev (extra external gearing)
    float maxCommand = 15.0f;       // RPM (stepper) or duty % (DC) — hard clamp in the driver
    float minCommand = 0.0f;        // lowest usable non-zero command (closed-loop lower clamp)
    float accel = 30.0f;            // RPM/s (stepper) or duty %/s (DC)
    bool invertDir = false;
    DecayMode decay = DecayMode::FAST;
    float minDuty = 0.0f;           // DC deadband: 0 < duty < minDuty drives the motor at 0
    uint16_t kickMs = 0;            // DC start kick duration (0 = off)
    float kickDuty = 80.0f;         // DC start kick duty %
    bool releaseWhenStopped = true; // de-energise coils / disable driver while stopped
};

// steps/s = rpm × stepsPerRev × microstep × gearRatio / 60
float rpmToStepsPerSec(float rpm, const MotorParams& m);
float stepsPerSecToRpm(float sps, const MotorParams& m);

// Theoretical syringe flow per driven-shaft RPM, µL/min per rpm:
//   area (mm²) × lead-screw pitch (mm/rev) = mm³/rev = µL/rev.
// Returns 0 if geometry is missing.
float syringeUlMinPerRpm(float innerDiameterMm, float leadscrewPitchMm);

// Moves `current` toward `target` by at most accel × dt. Used for ramping.
float rampToward(float current, float target, float accelPerS, float dtS);
