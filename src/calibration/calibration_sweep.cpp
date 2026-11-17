#include "calibration_sweep.h"
#include <cmath>

int linearCommands(float lo, float hi, int n, float* out) {
    if (n < 1) return 0;
    if (n > MAX_SWEEP_COMMANDS) n = MAX_SWEEP_COMMANDS;
    if (n == 1) {
        out[0] = hi;
        return 1;
    }
    for (int i = 0; i < n; ++i) out[i] = lo + (hi - lo) * i / (n - 1);
    return n;
}

const char* pointStatusName(PointStatus s) {
    switch (s) {
        case PointStatus::PENDING: return "pending";
        case PointStatus::ACTIVE: return "active";
        case PointStatus::OK: return "ok";
        case PointStatus::NOT_STEADY: return "not_steady";
        case PointStatus::INVALID_AIR: return "air";
        default: return "sensor";
    }
}

const char* sweepPhaseName(SweepPhase p) {
    switch (p) {
        case SweepPhase::IDLE: return "idle";
        case SweepPhase::ZERO_SETTLE: return "zero_settle";
        case SweepPhase::ZERO: return "zero";
        case SweepPhase::SETTLE: return "settle";
        case SweepPhase::STEADY: return "steady";
        case SweepPhase::AVERAGE: return "average";
        case SweepPhase::DONE: return "done";
        default: return "aborted";
    }
}

bool CalibrationSweep::running() const {
    return phase_ != SweepPhase::IDLE && phase_ != SweepPhase::DONE && phase_ != SweepPhase::ABORTED;
}

float CalibrationSweep::phaseDurationS() const {
    switch (phase_) {
        case SweepPhase::ZERO_SETTLE: return cfg_.zeroSettleS;
        case SweepPhase::ZERO: return cfg_.zeroS;
        case SweepPhase::SETTLE: return cfg_.settleS;
        case SweepPhase::STEADY: return cfg_.steadyTimeoutS;
        case SweepPhase::AVERAGE: return cfg_.averageS;
        default: return 0;
    }
}

void CalibrationSweep::reset() {
    *this = CalibrationSweep{};
}

bool CalibrationSweep::start(const SweepConfig& cfg, float nowS, const char** err) {
    if (running()) {
        if (err) *err = "sweep already running";
        return false;
    }
    if (cfg.nCommands < 1 || cfg.nCommands > MAX_SWEEP_COMMANDS) {
        if (err) *err = "1-20 command points required";
        return false;
    }
    const int reps = cfg.repeats < 1 ? 1 : cfg.repeats;
    const int total = cfg.nCommands * reps * (cfg.upDown ? 2 : 1);
    if (total > MAX_SWEEP_POINTS) {
        if (err) *err = "too many points (max 40 incl. repeats and descending pass)";
        return false;
    }
    for (int i = 0; i < cfg.nCommands; ++i) {
        if (!std::isfinite(cfg.commands[i]) || cfg.commands[i] < 0) {
            if (err) *err = "invalid command value";
            return false;
        }
    }
    if (!(cfg.settleS >= 0) || !(cfg.averageS > 0) || !(cfg.steadyTimeoutS >= 0)) {
        if (err) *err = "invalid timing";
        return false;
    }
    reset();
    cfg_ = cfg;
    cfg_.repeats = reps;
    for (int r = 0; r < reps; ++r) {
        for (int i = 0; i < cfg.nCommands; ++i) {
            SweepPoint& p = points_[nPoints_++];
            p.command = cfg.commands[i];
            p.pass = 0;
        }
        if (cfg.upDown) {
            for (int i = cfg.nCommands - 1; i >= 0; --i) {
                SweepPoint& p = points_[nPoints_++];
                p.command = cfg.commands[i];
                p.pass = 1;
            }
        }
    }
    const float win = cfg_.steadyWindowS > 1 ? cfg_.steadyWindowS : 1;
    steady_.configure(win);
    startS_ = nowS;
    if (cfg_.zeroFirst) {
        command_ = 0;
        enter(SweepPhase::ZERO_SETTLE, nowS);
    } else {
        beginPoint(0, nowS);
    }
    return true;
}

void CalibrationSweep::abort(float nowS) {
    if (!running()) return;
    if (idx_ >= 0 && idx_ < nPoints_ && points_[idx_].status == PointStatus::ACTIVE)
        points_[idx_].status = PointStatus::PENDING;
    command_ = 0;
    retrying_ = false;
    enter(SweepPhase::ABORTED, nowS);
}

void CalibrationSweep::enter(SweepPhase p, float nowS) {
    phase_ = p;
    phaseStart_ = nowS;
    if (p == SweepPhase::ZERO) {
        zeroSum_ = 0;
        zeroN_ = 0;
        zeroAir_ = false;
    } else if (p == SweepPhase::STEADY) {
        steady_.reset();
        notSteady_ = false;
    } else if (p == SweepPhase::AVERAGE) {
        n_ = 0;
        mean_ = m2_ = tempSum_ = 0;
        sawAir_ = false;
        invalidSamples_ = 0;
    }
}

void CalibrationSweep::beginPoint(int i, float nowS) {
    idx_ = i;
    points_[i].status = PointStatus::ACTIVE;
    command_ = points_[i].command;
    enter(SweepPhase::SETTLE, nowS);
}

void CalibrationSweep::finishPoint(float nowS, PointStatus st) {
    SweepPoint& p = points_[idx_];
    p.status = st;
    if (n_ > 0) {
        p.flow = static_cast<float>(mean_);
        p.std = n_ > 1 ? static_cast<float>(std::sqrt(m2_ / (n_ - 1))) : 0.0f;
        p.tempC = static_cast<float>(tempSum_ / n_);
    }
    p.n = n_;
    advance(nowS);
}

void CalibrationSweep::advance(float nowS) {
    if (retrying_) {
        retrying_ = false;
        command_ = 0;
        enter(SweepPhase::DONE, nowS);
        return;
    }
    const int next = idx_ + 1;
    if (next >= nPoints_) {
        command_ = 0;
        enter(SweepPhase::DONE, nowS);
        return;
    }
    beginPoint(next, nowS);
}

float CalibrationSweep::tick(float nowS, const SweepSample& s) {
    const float el = nowS - phaseStart_;
    switch (phase_) {
        case SweepPhase::ZERO_SETTLE:
            command_ = 0;
            if (el >= cfg_.zeroSettleS) enter(SweepPhase::ZERO, nowS);
            break;
        case SweepPhase::ZERO:
            command_ = 0;
            if (s.valid) {
                zeroSum_ += s.flowRaw;
                ++zeroN_;
                if (s.air) zeroAir_ = true;
            }
            if (el >= cfg_.zeroS) {
                zeroReady_ = true;
                zeroOk_ = zeroN_ > 0 && !zeroAir_;
                zeroOffset_ = zeroN_ > 0 ? static_cast<float>(zeroSum_ / zeroN_) : 0.0f;
                beginPoint(0, nowS);
            }
            break;
        case SweepPhase::SETTLE:
            if (el >= cfg_.settleS) enter(SweepPhase::STEADY, nowS);
            break;
        case SweepPhase::STEADY: {
            if (s.valid) steady_.add(nowS, s.flow);
            const bool windowFull = el >= cfg_.steadyWindowS;
            if (windowFull && steady_.count() > 2) {
                const float m = std::fabs(steady_.mean());
                const float slopePerMin = std::fabs(steady_.slopePerS() * 60.0f);
                const float slopeLim = std::fmax(cfg_.maxSlopePctPerMin / 100.0f * m, cfg_.slopeFloorUlMinPerMin);
                const float stdLim = std::fmax(cfg_.maxStdPct / 100.0f * m, cfg_.stdFloorUlMin);
                if (slopePerMin <= slopeLim && steady_.stddev() <= stdLim) {
                    enter(SweepPhase::AVERAGE, nowS);
                    break;
                }
            }
            if (el >= cfg_.steadyTimeoutS) {
                notSteady_ = true;   // proceed anyway; the point is flagged
                enter(SweepPhase::AVERAGE, nowS);
            }
            break;
        }
        case SweepPhase::AVERAGE:
            if (s.valid) {
                ++n_;
                const double d = s.flow - mean_;
                mean_ += d / n_;
                m2_ += d * (s.flow - mean_);
                tempSum_ += s.tempC;
                if (s.air) sawAir_ = true;
            } else {
                ++invalidSamples_;
            }
            if (el >= cfg_.averageS) {
                PointStatus st = PointStatus::OK;
                if (sawAir_) st = PointStatus::INVALID_AIR;
                else if (n_ == 0) st = PointStatus::INVALID_SENSOR;
                else if (notSteady_) st = PointStatus::NOT_STEADY;
                finishPoint(nowS, st);
            }
            break;
        default:
            command_ = 0;
            break;
    }
    return command_;
}

bool CalibrationSweep::takeZeroResult(float& offset, bool& okOut) {
    if (!zeroReady_) return false;
    zeroReady_ = false;
    offset = zeroOffset_;
    okOut = zeroOk_;
    return true;
}

bool CalibrationSweep::setReference(int i, float flowUlMin) {
    if (i < 0 || i >= nPoints_ || !std::isfinite(flowUlMin)) return false;
    if (points_[i].status == PointStatus::PENDING || points_[i].status == PointStatus::ACTIVE) return false;
    points_[i].ref = flowUlMin;
    return true;
}

bool CalibrationSweep::clearReference(int i) {
    if (i < 0 || i >= nPoints_) return false;
    points_[i].ref = NAN;
    return true;
}

bool CalibrationSweep::retryPoint(int i, float nowS) {
    if (running() || i < 0 || i >= nPoints_) return false;
    if (phase_ == SweepPhase::IDLE) return false;
    points_[i].flow = points_[i].std = points_[i].tempC = NAN;
    points_[i].n = 0;
    retrying_ = true;
    beginPoint(i, nowS);
    return true;
}

int CalibrationSweep::validPointCount() const {
    int n = 0;
    for (int i = 0; i < nPoints_; ++i)
        if (points_[i].valid() && std::isfinite(points_[i].effectiveFlow())) ++n;
    return n;
}

int CalibrationSweep::collect(float* x, float* y, int maxN) const {
    int n = 0;
    for (int i = 0; i < nPoints_ && n < maxN; ++i) {
        const SweepPoint& p = points_[i];
        if (!p.valid() || !std::isfinite(p.effectiveFlow())) continue;
        x[n] = p.command;
        y[n] = p.effectiveFlow();
        ++n;
    }
    return n;
}
