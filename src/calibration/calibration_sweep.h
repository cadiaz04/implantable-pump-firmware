#pragma once
// =============================================================================
// CalibrationSweep — firmware-orchestrated flow-vs-command sweep.
//
// Runs entirely on the ESP32 (a locked phone can't break it). Per point:
//   SETTLE (fixed time) → STEADY (rolling slope/std below thresholds, with a
//   timeout) → AVERAGE (mean/std/temp over the averaging time).
// An optional ZERO phase at the start (pump stopped) measures the sensor
// offset for tare. Air-in-line during averaging invalidates the point; the
// user can retry individual points or enter a manual (gravimetric /
// volumetric) reference that overrides the sensor value.
//
// Pure logic, unit tested natively.
// =============================================================================
#include <cmath>
#include <cstdint>
#include "util/window_stats.h"

constexpr int MAX_SWEEP_COMMANDS = 20;
constexpr int MAX_SWEEP_POINTS = 40;   // commands × repeats × (up/down ? 2 : 1)

struct SweepConfig {
    float commands[MAX_SWEEP_COMMANDS] = {};
    int nCommands = 0;
    float settleS = 60;
    float averageS = 60;
    int repeats = 1;
    bool upDown = false;            // add a descending pass (hysteresis)
    bool zeroFirst = true;
    float zeroSettleS = 15;
    float zeroS = 30;
    bool reverse = false;
    // steady-state detection
    float steadyWindowS = 20;
    float steadyTimeoutS = 120;
    float maxSlopePctPerMin = 3.0f;      // |slope| ≤ this % of |mean| per minute …
    float slopeFloorUlMinPerMin = 3.0f;  // … or this absolute floor, whichever is larger
    float maxStdPct = 25.0f;             // std ≤ this % of |mean| …
    float stdFloorUlMin = 20.0f;         // … or this absolute floor
};

// Builds `n` evenly spaced commands in [lo, hi].
int linearCommands(float lo, float hi, int n, float* out);

enum class PointStatus : uint8_t { PENDING, ACTIVE, OK, NOT_STEADY, INVALID_AIR, INVALID_SENSOR };
const char* pointStatusName(PointStatus s);

struct SweepPoint {
    float command = 0;
    uint8_t pass = 0;               // 0 = ascending, 1 = descending
    PointStatus status = PointStatus::PENDING;
    float flow = NAN;               // sensor mean, µL/min (zero-corrected)
    float std = NAN;
    float tempC = NAN;
    uint32_t n = 0;
    float ref = NAN;                // manual reference, µL/min (overrides flow)
    bool valid() const { return status == PointStatus::OK || status == PointStatus::NOT_STEADY || !std::isnan(ref); }
    float effectiveFlow() const { return std::isnan(ref) ? flow : ref; }
};

enum class SweepPhase : uint8_t { IDLE, ZERO_SETTLE, ZERO, SETTLE, STEADY, AVERAGE, DONE, ABORTED };
const char* sweepPhaseName(SweepPhase p);

struct SweepSample {
    bool valid = false;     // sensor OK and a new sample this tick
    float flowRaw = 0;      // without tare (for the zero phase)
    float flow = 0;         // tare-corrected
    float tempC = 0;
    bool air = false;
};

class CalibrationSweep {
public:
    bool start(const SweepConfig& cfg, float nowS, const char** err);
    void abort(float nowS);
    void reset();   // discard all data → IDLE

    // Call every tick. Returns the motor command to apply now.
    float tick(float nowS, const SweepSample& s);
    float command() const { return command_; }
    bool reverse() const { return cfg_.reverse; }

    // True while the motor is under sweep control.
    bool running() const;
    SweepPhase phase() const { return phase_; }
    int currentIndex() const { return idx_; }
    int pointCount() const { return nPoints_; }
    const SweepPoint& point(int i) const { return points_[i]; }
    const SweepConfig& config() const { return cfg_; }
    float phaseElapsedS(float nowS) const { return nowS - phaseStart_; }
    float phaseDurationS() const;   // nominal (STEADY reports the timeout)
    float startedAtS() const { return startS_; }

    // One-shot: true once after the ZERO phase completes; offset is the mean
    // raw flow with the pump stopped.
    bool takeZeroResult(float& offset, bool& okOut);

    // Manual reference / retry (allowed when DONE or ABORTED; references are
    // also allowed for already-completed points while running).
    bool setReference(int i, float flowUlMin);
    bool clearReference(int i);
    bool retryPoint(int i, float nowS);

    int validPointCount() const;
    // Copies valid (command, effectiveFlow) pairs; returns count.
    int collect(float* x, float* y, int maxN) const;

private:
    void enter(SweepPhase p, float nowS);
    void beginPoint(int i, float nowS);
    void finishPoint(float nowS, PointStatus st);
    void advance(float nowS);

    SweepConfig cfg_;
    SweepPoint points_[MAX_SWEEP_POINTS];
    int nPoints_ = 0;
    int idx_ = -1;
    SweepPhase phase_ = SweepPhase::IDLE;
    float phaseStart_ = 0;
    float startS_ = 0;
    float command_ = 0;
    bool retrying_ = false;
    bool notSteady_ = false;

    WindowStats steady_{20.0f};
    // AVERAGE accumulators (Welford)
    uint32_t n_ = 0;
    double mean_ = 0, m2_ = 0, tempSum_ = 0;
    bool sawAir_ = false;
    uint32_t invalidSamples_ = 0;

    // ZERO accumulators
    double zeroSum_ = 0;
    uint32_t zeroN_ = 0;
    bool zeroAir_ = false;
    bool zeroReady_ = false;
    bool zeroOk_ = false;
    float zeroOffset_ = 0;
};
