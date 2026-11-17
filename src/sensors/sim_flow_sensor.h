#pragma once
// SIM_MODE flow sensor: reads a SimPlant at the real sensor's 50 Hz rate,
// with the same fault semantics as Slf3sSensor (fault injection → FAULT
// after the same number of consecutive errors).
#include "sensors/sensor.h"
#include "sensors/sim_plant.h"

class SimFlowSensor : public FlowSensor {
public:
    static constexpr uint32_t kFaultAfter = 10;
    SimFlowSensor(SimPlant& plant, int8_t channel) : plant_(plant), channel_(channel) {}
    bool begin() override {
        status_ = SensorStatus::INIT;
        return true;
    }
    // The channel advances the plant (SENSOR_POLL_PERIOD_MS); this just picks
    // up each new plant sample.
    bool poll(uint32_t nowMs) override {
        if (plant_.sampleCount() == seen_) return false;
        seen_ = plant_.sampleCount();
        flow_ = plant_.measured();
        if (plant_.fault()) {
            ++err_.crc;
            if (++err_.consecutive >= kFaultAfter) status_ = SensorStatus::FAULT;
            return false;
        }
        if (status_ == SensorStatus::FAULT) ++err_.reinits;
        err_.consecutive = 0;
        status_ = SensorStatus::OK;
        sampleMs_ = nowMs;
        return true;
    }
    SensorStatus status() const override { return status_; }
    const SensorErrors& errors() const override { return err_; }
    int signalCount() const override { return 0; }
    SignalInfo signalInfo(int) const override { return {"", "", "", channel_}; }
    float signalValue(int) const override { return 0; }
    float flowUlMin() const override { return flow_; }
    float tempC() const override { return plant_.tempC(); }
    uint16_t flags() const override { return plant_.bubbleActive() ? 1 : 0; }
    bool airInLine() const override { return plant_.bubbleActive(); }
    bool highFlow() const override { return false; }
    uint32_t sampleTimeMs() const override { return sampleMs_; }

private:
    SimPlant& plant_;
    int8_t channel_;
    SensorStatus status_ = SensorStatus::INIT;
    SensorErrors err_;
    uint32_t seen_ = 0, sampleMs_ = 0;
    float flow_ = 0;
};
