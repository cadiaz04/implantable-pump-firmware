#pragma once
// =============================================================================
// MotorDriver — common interface for every motor driver.
//
// Hardware-facing only: no control logic here. The command variable is RPM
// at the driven shaft for steppers and PWM duty % for DC motors. The driver
// itself enforces the profile's max-command clamp (defence in depth — upper
// layers already reject out-of-range requests) and ramps with the profile's
// acceleration so the pump never jerks. Reversing ramps through zero.
// =============================================================================
#include "motors/motor_types.h"

class MotorDriver {
public:
    explicit MotorDriver(DriverType t) : type_(t) {}
    virtual ~MotorDriver() = default;

    virtual bool begin() = 0;   // claim pins / peripherals
    virtual void end() = 0;     // release pins, outputs safe

    void setParams(const MotorParams& p) { params_ = p; }
    const MotorParams& params() const { return params_; }

    // Target command; clamped to [0, maxCommand] here regardless of caller.
    void setTarget(float command, bool reverse);
    // Immediate stop, no ramp (used for STOP / E-STOP).
    void stop();
    // Driver output stage enable (STEP/DIR EN, TB6612 STBY…). E-STOP disables.
    void enable(bool en);
    bool enabled() const { return enabled_; }

    // Advance the ramp by dt and write the hardware. Real-time task only.
    void update(float dtS);

    float currentCommand() const { return current_; }
    float targetCommand() const { return target_; }
    bool direction() const { return curReverse_; }
    DriverType type() const { return type_; }
    const char* commandUnit() const { return commandUnitFor(type_); }
    virtual float stepRateHz() const { return 0; }

protected:
    // Write `command` (already ramped/clamped) to the hardware.
    virtual void apply(float command, bool reverse, float dtS) = 0;
    virtual void applyEnable(bool en) = 0;
    float maxAllowed() const;

    DriverType type_;
    MotorParams params_;
    float current_ = 0, target_ = 0;
    bool reverse_ = false, curReverse_ = false;
    bool enabled_ = true;
};
