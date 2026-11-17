#pragma once
// Browser heartbeat watchdog. Any client's heartbeat (WebSocket "hb", REST
// /api/hb, or starting a run) feeds it. Pure logic, unit tested natively.
#include <cstdint>

class HeartbeatWatchdog {
public:
    void feed(uint32_t nowMs) {
        lastMs_ = nowMs;
        fed_ = true;
    }
    bool everFed() const { return fed_; }
    float ageS(uint32_t nowMs) const { return fed_ ? (nowMs - lastMs_) / 1000.0f : 1e9f; }
    bool expired(uint32_t nowMs, float timeoutS) const { return ageS(nowMs) > timeoutS; }

private:
    uint32_t lastMs_ = 0;
    bool fed_ = false;
};
