#include "flow_controller.h"
#include <cmath>

void FlowController::configure(const FlowControlConfig& c) {
    cfg_ = c;
    if (!(cfg_.loopHz >= 0.2f)) cfg_.loopHz = 0.2f;
    if (cfg_.loopHz > 10.0f) cfg_.loopHz = 10.0f;
    applyLimits();
}

void FlowController::applyLimits() {
    PidConfig p;
    p.kp = cfg_.kp;
    p.ki = cfg_.ki;
    p.kd = cfg_.kd;
    const float base = std::fabs(sp_);
    const float outLim = std::fmax(cfg_.outputLimitPct / 100.0f * base, cfg_.limitFloorUlMin);
    const float iLim = std::fmax(cfg_.integralLimitPct / 100.0f * base, cfg_.limitFloorUlMin);
    p.outMin = -outLim;
    p.outMax = outLim;
    p.iMin = -iLim;
    p.iMax = iLim;
    pid_.configure(p);
}

bool FlowController::setSetpoint(float spUlMin) {
    if (!std::isfinite(spUlMin) || spUlMin < cfg_.minFlowUlMin || spUlMin > cfg_.maxFlowUlMin) {
        // Reject outright rather than silently clamping — fail closed.
        safe_ = false;
        sp_ = 0;
        cmd_ = 0;
        return false;
    }
    safe_ = true;
    sp_ = spUlMin;
    applyLimits();
    first_ = true;
    return true;
}

void FlowController::reset() {
    pid_.reset(0);
    acc_ = 0;
    first_ = true;
    satDir_ = 0;
    corr_ = 0;
}

float FlowController::update(float dtS, float measuredUlMin, bool measuredValid) {
    if (!safe_ || model_ == nullptr) {
        cmd_ = 0;
        return 0;
    }
    if (sp_ <= 0) {
        cmd_ = ff_ = corr_ = 0;
        return 0;
    }
    acc_ += dtS;
    const float period = 1.0f / cfg_.loopHz;
    if (!first_ && acc_ < period) return cmd_;
    const float loopDt = first_ ? period : acc_;
    acc_ = 0;
    first_ = false;

    if (measuredValid && std::isfinite(measuredUlMin)) {
        corr_ = pid_.update(sp_, measuredUlMin, loopDt, satDir_);
    } else {
        corr_ = pid_.integral();   // hold
    }
    ff_ = model_->commandFor(sp_, cfg_.minCommand, cfg_.maxCommand);
    float c = model_->commandFor(sp_ + corr_, cfg_.minCommand, cfg_.maxCommand);
    satDir_ = 0;
    if (c >= cfg_.maxCommand) {
        c = cfg_.maxCommand;
        satDir_ = +1;
    } else if (c <= cfg_.minCommand) {
        c = cfg_.minCommand;
        satDir_ = -1;
    }
    cmd_ = c;
    return cmd_;
}
