#include "mprls_sensor.h"
#include <Arduino.h>

bool MprlsSensor::begin() {
    phase_ = Phase::POWER_UP;
    phaseMs_ = millis();
    status_ = SensorStatus::INIT;
    return true;
}

bool MprlsSensor::trigger(uint32_t nowMs) {
    wire_.beginTransmission(mprls::I2C_ADDRESS);
    wire_.write(mprls::CMD_MEASURE, sizeof(mprls::CMD_MEASURE));
    if (wire_.endTransmission() != 0) {
        fail(false, nowMs);
        return false;
    }
    triggerMs_ = nowMs;
    phase_ = Phase::CONVERTING;
    return true;
}

void MprlsSensor::fail(bool frameError, uint32_t nowMs) {
    if (frameError) ++err_.crc;   // "crc" counter = bad frames (the MPR has no CRC)
    else ++err_.i2c;
    ++err_.consecutive;
    if (err_.consecutive >= kFaultAfter && status_ != SensorStatus::FAULT) {
        status_ = SensorStatus::FAULT;
        phase_ = Phase::BACKOFF;
        phaseMs_ = nowMs;
    }
}

bool MprlsSensor::poll(uint32_t nowMs) {
    switch (phase_) {
        case Phase::POWER_UP:
            if (nowMs - phaseMs_ < mprls::POWER_UP_MS) return false;
            if (!trigger(nowMs)) phaseMs_ = nowMs;   // retry later (fail() counts it)
            return false;
        case Phase::BACKOFF:
            if (nowMs - phaseMs_ < 1000) return false;
            ++err_.reinits;
            if (!trigger(nowMs)) phaseMs_ = nowMs;
            return false;
        case Phase::CONVERTING:
            break;
    }
    if (nowMs - triggerMs_ < kPeriodMs) return false;   // ≥ 5 ms conversion, paced to 50 Hz
    uint8_t buf[mprls::FRAME_BYTES];
    const size_t got = wire_.requestFrom(static_cast<int>(mprls::I2C_ADDRESS), static_cast<int>(mprls::FRAME_BYTES));
    if (got != mprls::FRAME_BYTES) {
        while (wire_.available()) wire_.read();
        fail(false, nowMs);
        trigger(nowMs);
        return false;
    }
    for (uint8_t& b : buf) b = wire_.read();
    mprls::Sample s;
    const mprls::Result r = mprls::parseFrame(buf, sizeof(buf), s);
    switch (r) {
        case mprls::Result::BUSY:
            // Still converting: read again next poll without re-triggering.
            if (++busyCount_ > 10) {
                busyCount_ = 0;
                fail(true, nowMs);
                trigger(nowMs);
            } else {
                triggerMs_ = nowMs - kPeriodMs + 2;
            }
            return false;
        case mprls::Result::INTEGRITY_FAIL:
            // Computed only at sensor power-up: needs a power cycle.
            s_.status = s.status;
            integrityFail_ = true;
            status_ = SensorStatus::FAULT;
            phase_ = Phase::BACKOFF;
            phaseMs_ = nowMs;
            return false;
        case mprls::Result::BAD_FRAME:
            fail(true, nowMs);
            trigger(nowMs);
            return false;
        case mprls::Result::OK:
        case mprls::Result::SATURATED:
            break;
    }
    busyCount_ = 0;
    err_.consecutive = 0;
    integrityFail_ = false;
    status_ = SensorStatus::OK;
    saturated_ = r == mprls::Result::SATURATED;
    if (!saturated_) s_ = s;
    else s_.status = s.status;
    sampleMs_ = nowMs;
    trigger(nowMs);
    return true;
}
