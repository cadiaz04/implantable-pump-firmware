#pragma once
// =============================================================================
// SimPlant — simulated pump + tubing + flow sensor for SIM_MODE and tests.
//
//   true flow  = gain·u + quad·u² (u = command − deadband, 0 below deadband)
//   measured   = first-order lag (tau) of true flow after a dead time,
//                + sensor zero offset + white noise + optional pulsation
//   pressure   = atmosphere + line resistance × pump flow (lagged), + noise
//                (absolute, like the MPRLS)
//
// Optional bubble (air-in-line), occlusion and sensor-fault injection.
// Pure logic, deterministic given a seed.
// =============================================================================
#include <cstdint>

struct SimPlantParams {
    float gain = 20.0f;          // µL/min per command unit above deadband
    float quad = 0.0f;           // µL/min per unit²
    float deadband = 0.0f;       // command units with no flow (DC stiction)
    float tauS = 4.0f;           // thermal/tubing lag
    float deadTimeS = 2.0f;      // transport delay
    float noiseUlMin = 5.0f;     // per-sample noise σ
    float zeroOffsetUlMin = 8.0f;
    float pulsationPct = 0.0f;   // diaphragm pumps: ±% of flow
    float pulsationHzPerCmd = 0.0f;
    float tempC = 23.0f;
    // pressure
    float resistanceKpaPerUlMin = 0.02f;   // line + catheter resistance
    float pressureTauS = 1.5f;             // line compliance
    float pressureNoiseKpa = 0.03f;
    float atmosphereKpa = 100.8f;          // deliberately not 101.325: forces a real tare
    float occlusionFactor = 40.0f;         // resistance multiplier when occluded
};

class SimPlant {
public:
    explicit SimPlant(uint32_t seed = 12345) : rng_(seed ? seed : 1) {}
    void setParams(const SimPlantParams& p) { p_ = p; }
    const SimPlantParams& params() const { return p_; }

    void setCommand(float command, bool reverse) {
        cmd_ = command;
        reverse_ = reverse;
    }
    // Advance the model by dt seconds; returns the measured flow (µL/min).
    float step(float dtS);
    float trueFlow() const { return trueFlow_; }
    float steadyFlowFor(float command) const;   // noiseless static curve
    float measured() const { return measured_; }
    float tempC() const { return temp_; }

    void injectBubble(float seconds) { bubbleS_ = seconds; }
    bool bubbleActive() const { return bubbleS_ > 0; }
    void setFault(bool f) { fault_ = f; }
    bool fault() const { return fault_; }

    // Downstream occlusion: pressure climbs, delivered (measured) flow collapses.
    void setOcclusion(bool o) { occluded_ = o; }
    bool occluded() const { return occluded_; }
    float pressureAbsKpa() const { return pressure_; }
    void setPressureFault(bool f) { pressureFault_ = f; }
    bool pressureFault() const { return pressureFault_; }
    static constexpr float kPressureFullScaleKpa = 25.0f * 6.894757f;   // 0-25 psia sensor
    // Increments once per step(); sensors use it to detect new samples.
    uint32_t sampleCount() const { return samples_; }

private:
    float gaussian();

    SimPlantParams p_;
    uint32_t rng_;
    float cmd_ = 0;
    bool reverse_ = false;
    float trueFlow_ = 0;
    float lagged_ = 0;
    float measured_ = 0;
    float temp_ = 23.0f;
    float t_ = 0;
    float phase_ = 0;
    float bubbleS_ = 0;
    bool fault_ = false;
    bool occluded_ = false;
    bool pressureFault_ = false;
    float pressureLag_ = 0;
    float pressure_ = 101.0f;
    uint32_t samples_ = 0;

    // Dead-time line, 20 Hz resolution, up to 10 s.
    static constexpr int kDelaySlots = 200;
    static constexpr float kDelayDt = 0.05f;
    float delay_[kDelaySlots] = {};
    int delayHead_ = 0;
    float delayAcc_ = 0;
};
