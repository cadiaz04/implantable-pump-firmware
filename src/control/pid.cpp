#include "pid.h"

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

float Pid::update(float setpoint, float measurement, float dtS, int satDir) {
    const float e = setpoint - measurement;
    p_ = cfg_.kp * e;
    d_ = 0;
    if (havePrev_ && dtS > 0) d_ = -cfg_.kd * (measurement - prevMeas_) / dtS;
    prevMeas_ = measurement;
    havePrev_ = true;

    float candidate = clampf(integral_ + cfg_.ki * e * dtS, cfg_.iMin, cfg_.iMax);
    const float unsat = p_ + candidate + d_;
    const bool outHigh = unsat > cfg_.outMax || satDir > 0;
    const bool outLow = unsat < cfg_.outMin || satDir < 0;
    // Only let the integrator move if it isn't pushing further into saturation.
    if ((outHigh && candidate > integral_) || (outLow && candidate < integral_)) candidate = integral_;
    integral_ = candidate;
    return clampf(p_ + integral_ + d_, cfg_.outMin, cfg_.outMax);
}
