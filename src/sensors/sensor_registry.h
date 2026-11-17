#pragma once
// Registry of *additional* sensors (pressure, …) beyond each channel's flow
// sensor. Everything registered here is polled by the real-time task and its
// signals appear automatically in telemetry, live plots and CSV exports.
#include "sensors/sensor.h"

class SensorRegistry {
public:
    static constexpr int kMax = 8;
    bool add(Sensor* s) {
        if (n_ >= kMax || !s) return false;
        s_[n_++] = s;
        return true;
    }
    int count() const { return n_; }
    Sensor* at(int i) const { return s_[i]; }
    void beginAll() {
        for (int i = 0; i < n_; ++i) s_[i]->begin();
    }
    void pollAll(uint32_t nowMs) {
        for (int i = 0; i < n_; ++i) s_[i]->poll(nowMs);
    }

private:
    Sensor* s_[kMax] = {};
    int n_ = 0;
};

extern SensorRegistry g_sensors;
