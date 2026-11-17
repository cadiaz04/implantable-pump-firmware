#pragma once
// SIM_MODE pressure sensor: reads the SimPlant's absolute pressure at 50 Hz
// with the same status semantics as MprlsSensor (saturation above 25 psia,
// FAULT after 10 consecutive errors when fault injection is on).
#include "sensors/sensor.h"
#include "sensors/sim_plant.h"

class SimPressureSensor : public PressureSensor {
public:
    static constexpr uint32_t kFaultAfter = 10;
    SimPressureSensor(SimPlant& plant, int8_t channel) : plant_(plant), channel_(channel) {}
    bool begin() override {
        status_ = SensorStatus::INIT;
        return true;
    }
    bool poll(uint32_t nowMs) override {
        if (plant_.sampleCount() == seen_) return false;
        seen_ = plant_.sampleCount();
        if (plant_.pressureFault()) {
            ++err_.i2c;
            if (++err_.consecutive >= kFaultAfter) status_ = SensorStatus::FAULT;
            return false;
        }
        err_.consecutive = 0;
        status_ = SensorStatus::OK;
        saturated_ = plant_.pressureAbsKpa() > SimPlant::kPressureFullScaleKpa;
        if (!saturated_) abs_ = plant_.pressureAbsKpa();
        sampleMs_ = nowMs;
        return true;
    }
    SensorStatus status() const override { return status_; }
    const SensorErrors& errors() const override { return err_; }
    int signalCount() const override { return 0; }
    SignalInfo signalInfo(int) const override { return {"", "", "", channel_}; }
    float signalValue(int) const override { return 0; }
    float absKpa() const override { return abs_; }
    bool saturated() const override { return saturated_; }
    uint8_t statusByte() const override { return saturated_ ? 0x41 : 0x40; }
    uint32_t sampleTimeMs() const override { return sampleMs_; }

private:
    SimPlant& plant_;
    int8_t channel_;
    SensorStatus status_ = SensorStatus::INIT;
    SensorErrors err_;
    uint32_t seen_ = 0, sampleMs_ = 0;
    float abs_ = 0;
    bool saturated_ = false;
};
