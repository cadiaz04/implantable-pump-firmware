#pragma once
// =============================================================================
// MprlsSensor — Honeywell MPRLS0025PA00001A (SparkFun Qwiic MicroPressure)
// on the channel's I²C bus, shared with the SLF3S (0x08 vs 0x18).
//
// Non-blocking, pipelined from the real-time task: trigger a conversion
// (0xAA 0x00 0x00), read the 4-byte result ≥ 20 ms later, trigger the next
// one immediately → 50 Hz with no waiting. The channel begins the bus; this
// class never calls Wire.begin()/end().
// =============================================================================
#include <Wire.h>
#include "sensors/mprls_protocol.h"
#include "sensors/sensor.h"

class MprlsSensor : public PressureSensor {
public:
    static constexpr uint32_t kFaultAfter = 10;
    static constexpr uint32_t kPeriodMs = 20;

    MprlsSensor(TwoWire& wire, int8_t channel) : wire_(wire), channel_(channel) {}
    bool begin() override;
    bool poll(uint32_t nowMs) override;
    SensorStatus status() const override { return status_; }
    const SensorErrors& errors() const override { return err_; }
    int signalCount() const override { return 0; }   // the channel exposes gauge pressure
    SignalInfo signalInfo(int) const override { return {"", "", "", channel_}; }
    float signalValue(int) const override { return 0; }

    float absKpa() const override { return s_.kPaAbs; }
    bool saturated() const override { return saturated_; }
    uint8_t statusByte() const override { return s_.status; }
    uint32_t sampleTimeMs() const override { return sampleMs_; }

private:
    enum class Phase : uint8_t { POWER_UP, CONVERTING, BACKOFF };
    bool trigger(uint32_t nowMs);
    void fail(bool frameError, uint32_t nowMs);

    TwoWire& wire_;
    int8_t channel_;
    SensorStatus status_ = SensorStatus::INIT;
    SensorErrors err_;
    Phase phase_ = Phase::POWER_UP;
    uint32_t phaseMs_ = 0;
    uint32_t triggerMs_ = 0;
    uint32_t sampleMs_ = 0;
    uint32_t busyCount_ = 0;
    bool saturated_ = false;
    bool integrityFail_ = false;
    mprls::Sample s_;
};
