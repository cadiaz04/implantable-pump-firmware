#pragma once
// =============================================================================
// FlowController — closed-loop flow control: feed-forward + PI(D) correction.
//
//   command = f⁻¹( setpoint + correction ),  correction = PID(setpoint − measured)
//
// The PID works in FLOW units (µL/min): its output is a flow correction that
// is pushed through the pump's inverse model. That makes the gains
// dimensionless (kp) and 1/s (ki), so the same gains work across pumps with
// very different µL/min-per-RPM slopes.
//
// SAFETY: setSetpoint() REJECTS out-of-range or NaN/inf setpoints (returns
// false, isSafe() == false, command 0) instead of clamping. The caller must
// stop the motor when isSafe() is false. Output command is clamped to the
// pump's [minCommand, maxCommand] (actuator saturation, not setpoint
// clamping), with integrator anti-windup.
//
// Deliberately has zero Arduino dependencies (unit tested natively).
// =============================================================================
#include "config.h"
#include "control/flow_model.h"
#include "control/pid.h"

struct FlowControlConfig {
    float kp = 0.3f;
    float ki = 0.05f;           // 1/s
    float kd = 0.0f;            // s
    float loopHz = 1.0f;        // 0.2 – 10 Hz
    float outputLimitPct = 100; // |correction| ≤ this % of setpoint (with a floor)
    float integralLimitPct = 100;
    float limitFloorUlMin = 20; // floor for the two limits at very low setpoints
    float minCommand = 0;
    float maxCommand = 0;
    float minFlowUlMin = MIN_FLOW_UL_MIN;
    float maxFlowUlMin = MAX_FLOW_UL_MIN;
};

class FlowController {
public:
    void configure(const FlowControlConfig& c);
    const FlowControlConfig& config() const { return cfg_; }
    void setModel(const FlowModel* m) { model_ = m; }   // non-owning
    const FlowModel* model() const { return model_; }

    // Returns false (and fails closed) for NaN/inf/out-of-range setpoints.
    bool setSetpoint(float spUlMin);
    void reset();   // clears integrator; next update runs immediately

    // Call every tick; internally runs at loopHz. Returns the motor command.
    // measuredValid = false holds the integrator (pure feed-forward + held
    // correction).
    float update(float dtS, float measuredUlMin, bool measuredValid);

    bool isSafe() const { return safe_; }
    float setpoint() const { return sp_; }
    float command() const { return cmd_; }
    float feedForward() const { return ff_; }
    float correction() const { return corr_; }
    const Pid& pid() const { return pid_; }

private:
    void applyLimits();

    FlowControlConfig cfg_;
    const FlowModel* model_ = nullptr;
    Pid pid_;
    float sp_ = 0;
    bool safe_ = false;
    float acc_ = 0;
    bool first_ = true;
    float cmd_ = 0, ff_ = 0, corr_ = 0;
    int satDir_ = 0;
};
