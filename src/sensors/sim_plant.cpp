#include "sim_plant.h"
#include <cmath>

float SimPlant::gaussian() {
    // xorshift32 + Box-Muller
    auto next = [this]() {
        rng_ ^= rng_ << 13;
        rng_ ^= rng_ >> 17;
        rng_ ^= rng_ << 5;
        return (rng_ & 0xFFFFFF) / 16777216.0f;
    };
    float u1 = next();
    if (u1 < 1e-7f) u1 = 1e-7f;
    const float u2 = next();
    return std::sqrt(-2.0f * std::log(u1)) * std::cos(6.2831853f * u2);
}

float SimPlant::steadyFlowFor(float command) const {
    const float u = command - p_.deadband;
    if (u <= 0) return 0.0f;
    return p_.gain * u + p_.quad * u * u;
}

float SimPlant::step(float dtS) {
    if (dtS <= 0) return measured_;
    t_ += dtS;
    trueFlow_ = steadyFlowFor(std::fabs(cmd_)) * (reverse_ ? -1.0f : 1.0f);

    // Dead time: push into the delay line at fixed resolution.
    const int slots = static_cast<int>(p_.deadTimeS / kDelayDt);
    const int n = slots < 1 ? 1 : (slots >= kDelaySlots ? kDelaySlots - 1 : slots);
    delayAcc_ += dtS;
    while (delayAcc_ >= kDelayDt) {
        delayAcc_ -= kDelayDt;
        delay_[delayHead_] = trueFlow_;
        delayHead_ = (delayHead_ + 1) % kDelaySlots;
    }
    const float delayed = delay_[(delayHead_ - n + kDelaySlots) % kDelaySlots];

    // First-order lag.
    const float a = p_.tauS > 0 ? 1.0f - std::exp(-dtS / p_.tauS) : 1.0f;
    lagged_ += a * (delayed - lagged_);

    float flow = lagged_;
    if (p_.pulsationPct > 0 && std::fabs(cmd_) > p_.deadband) {
        phase_ += 6.2831853f * p_.pulsationHzPerCmd * std::fabs(cmd_) * dtS;
        if (phase_ > 6.2831853f) phase_ -= 6.2831853f;
        flow += lagged_ * (p_.pulsationPct / 100.0f) * std::sin(phase_);
    }
    if (bubbleS_ > 0) {
        bubbleS_ -= dtS;
        flow *= 0.1f;   // thermal sensor reads near-zero with air over the chip
    }
    if (occluded_) flow *= 0.05f;   // blocked line: almost nothing gets through
    temp_ = p_.tempC + 0.3f * std::sin(t_ / 600.0f);
    measured_ = flow + p_.zeroOffsetUlMin + p_.noiseUlMin * gaussian();

    // Pressure follows the pump side (no transport delay), through the line
    // compliance. Reverse flow pulls a (small) vacuum.
    const float r = p_.resistanceKpaPerUlMin * (occluded_ ? p_.occlusionFactor : 1.0f);
    const float ap = p_.pressureTauS > 0 ? 1.0f - std::exp(-dtS / p_.pressureTauS) : 1.0f;
    pressureLag_ += ap * (r * trueFlow_ - pressureLag_);
    pressure_ = p_.atmosphereKpa + 0.15f * std::sin(t_ / 900.0f) + pressureLag_ + p_.pressureNoiseKpa * gaussian();
    ++samples_;
    return measured_;
}
