#pragma once
// =============================================================================
// Slf3sSensor — Sensirion SLF3S-1300F on a dedicated I²C bus.
//
// Non-blocking state machine driven from the real-time task:
//   soft reset → wait 25 ms → start continuous (water) → wait 60 ms → read
//   9-byte frames every SENSOR_POLL_PERIOD_MS.
// CRC / I²C failures are counted; after kFaultAfter consecutive failures the
// sensor is FAULT and re-initialised (with I²C bus recovery) once a second.
//
// The channel owns the bus (Wire.begin/end) because it may be shared with a
// pressure sensor. On a shared bus the general-call soft reset is replaced by
// "stop continuous measurement": a general call reaches every device.
// =============================================================================
#include <Wire.h>
#include "sensors/sensor.h"
#include "sensors/slf3s_protocol.h"

class Slf3sSensor : public FlowSensor {
public:
    static constexpr uint32_t kFaultAfter = 10;

    Slf3sSensor(TwoWire& wire, int sda, int scl, int8_t channel, bool sharedBus);
    ~Slf3sSensor() override;
    bool begin() override;
    bool poll(uint32_t nowMs) override;
    SensorStatus status() const override { return status_; }
    const SensorErrors& errors() const override { return err_; }

    int signalCount() const override { return 0; }   // channel exposes flow/temp
    SignalInfo signalInfo(int) const override { return {"", "", "", channel_}; }
    float signalValue(int) const override { return 0; }

    float flowUlMin() const override { return s_.flowUlMin; }
    float tempC() const override { return s_.tempC; }
    uint16_t flags() const override { return s_.flags; }
    bool airInLine() const override { return s_.flags & slf3s::FLAG_AIR_IN_LINE; }
    bool highFlow() const override { return s_.flags & slf3s::FLAG_HIGH_FLOW; }
    uint32_t sampleTimeMs() const override { return sampleMs_; }
    uint32_t productId() const { return productId_; }

private:
    enum class Phase : uint8_t { RESET_WAIT, START_WAIT, RUN, BACKOFF };
    bool sendCommand(uint16_t cmd);
    void reset();
    void recoverBus();
    void fail(bool crc, uint32_t nowMs);
    void readProductId();

    TwoWire& wire_;
    int sda_, scl_;
    int8_t channel_;
    SensorStatus status_ = SensorStatus::INIT;
    SensorErrors err_;
    Phase phase_ = Phase::RESET_WAIT;
    uint32_t phaseMs_ = 0;
    uint32_t lastReadMs_ = 0;
    uint32_t sampleMs_ = 0;
    uint32_t productId_ = 0;
    slf3s::Sample s_;
    bool sharedBus_ = false;
};
