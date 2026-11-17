#pragma once
// A pump's flow-vs-command model used for feed-forward / open-loop control.
#include "calibration/curve_fit.h"

class FlowModel {
public:
    virtual ~FlowModel() = default;
    virtual float flowAt(float command) const = 0;
    // Command in [lo, hi] that should produce `flow`.
    virtual float commandFor(float flow, float lo, float hi) const = 0;
};

// From an accepted calibration.
class CurveFlowModel : public FlowModel {
public:
    CurveFlowModel() = default;
    explicit CurveFlowModel(const CurveFit& f) : fit_(f) {}
    void set(const CurveFit& f) { fit_ = f; }
    const CurveFit& fit() const { return fit_; }
    float flowAt(float command) const override { return fit_.eval(command); }
    float commandFor(float flow, float lo, float hi) const override { return fit_.inverse(flow, lo, hi); }

private:
    CurveFit fit_;
};

// flow = k · command (theoretical syringe geometry, or a user-supplied
// initial command when no calibration exists).
class ProportionalFlowModel : public FlowModel {
public:
    explicit ProportionalFlowModel(float k = 0) : k_(k) {}
    void setGain(float k) { k_ = k; }
    float gain() const { return k_; }
    float flowAt(float command) const override { return k_ * command; }
    float commandFor(float flow, float lo, float hi) const override {
        if (!(k_ > 0)) return lo;
        const float c = flow / k_;
        return c < lo ? lo : (c > hi ? hi : c);
    }

private:
    float k_;
};
