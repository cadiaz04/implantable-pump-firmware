#include "slf3s_sensor.h"
#include <Arduino.h>
#include "config.h"
#include "util/crc8.h"

Slf3sSensor::Slf3sSensor(TwoWire& wire, int sda, int scl, int8_t channel, bool sharedBus)
    : wire_(wire), sda_(sda), scl_(scl), channel_(channel), sharedBus_(sharedBus) {}

Slf3sSensor::~Slf3sSensor() { sendCommand(slf3s::CMD_STOP_CONTINUOUS); }

bool Slf3sSensor::begin() {
    reset();
    phase_ = Phase::RESET_WAIT;
    phaseMs_ = millis();
    status_ = SensorStatus::INIT;
    return true;
}

bool Slf3sSensor::sendCommand(uint16_t cmd) {
    uint8_t b[2];
    slf3s::encodeCommand(cmd, b);
    wire_.beginTransmission(slf3s::I2C_ADDRESS);
    wire_.write(b, 2);
    return wire_.endTransmission() == 0;
}

void Slf3sSensor::reset() {
    if (sharedBus_) {
        // Leaves continuous mode (e.g. after an ESP32-only reboot) without
        // touching the other device on the bus.
        sendCommand(slf3s::CMD_STOP_CONTINUOUS);
        return;
    }
    // General-call reset; this sensor is alone on its bus.
    wire_.beginTransmission(slf3s::GENERAL_CALL_ADDRESS);
    wire_.write(slf3s::GENERAL_CALL_RESET);
    wire_.endTransmission();
}

void Slf3sSensor::recoverBus() {
    // Clock out a slave that is holding SDA low, then issue a STOP.
    wire_.end();
    pinMode(sda_, INPUT_PULLUP);
    pinMode(scl_, OUTPUT_OPEN_DRAIN);
    for (int i = 0; i < 9 && digitalRead(sda_) == LOW; ++i) {
        digitalWrite(scl_, LOW);
        delayMicroseconds(5);
        digitalWrite(scl_, HIGH);
        delayMicroseconds(5);
    }
    pinMode(sda_, OUTPUT_OPEN_DRAIN);
    digitalWrite(sda_, LOW);
    delayMicroseconds(5);
    digitalWrite(scl_, HIGH);
    delayMicroseconds(5);
    digitalWrite(sda_, HIGH);
    delayMicroseconds(5);
    wire_.begin(sda_, scl_, 400000);
    wire_.setTimeOut(10);
}

void Slf3sSensor::readProductId() {
    if (!sendCommand(slf3s::CMD_READ_PRODUCT_ID_1)) return;
    if (!sendCommand(slf3s::CMD_READ_PRODUCT_ID_2)) return;
    uint8_t b[6];
    if (wire_.requestFrom(static_cast<int>(slf3s::I2C_ADDRESS), 6) != 6) return;
    for (uint8_t& x : b) x = wire_.read();
    if (crc8Sensirion(b, 2) != b[2] || crc8Sensirion(b + 3, 2) != b[5]) return;
    productId_ = (uint32_t(b[0]) << 24) | (uint32_t(b[1]) << 16) | (uint32_t(b[3]) << 8) | b[4];
}

void Slf3sSensor::fail(bool crc, uint32_t nowMs) {
    if (crc) ++err_.crc;
    else ++err_.i2c;
    ++err_.consecutive;
    if (err_.consecutive >= kFaultAfter && status_ != SensorStatus::FAULT) {
        status_ = SensorStatus::FAULT;
        phase_ = Phase::BACKOFF;
        phaseMs_ = nowMs;
    }
}

bool Slf3sSensor::poll(uint32_t nowMs) {
    switch (phase_) {
        case Phase::RESET_WAIT:
            if (nowMs - phaseMs_ < slf3s::POWER_UP_MS) return false;
            readProductId();
            if (!sendCommand(slf3s::CMD_START_CONTINUOUS_WATER)) {
                fail(false, nowMs);
                if (status_ != SensorStatus::FAULT) phaseMs_ = nowMs;   // retry after another wait
                return false;
            }
            phase_ = Phase::START_WAIT;
            phaseMs_ = nowMs;
            return false;
        case Phase::START_WAIT:
            if (nowMs - phaseMs_ < slf3s::FIRST_SAMPLE_MS) return false;
            phase_ = Phase::RUN;
            lastReadMs_ = nowMs - SENSOR_POLL_PERIOD_MS;
            return false;
        case Phase::BACKOFF:
            if (nowMs - phaseMs_ < 1000) return false;
            ++err_.reinits;
            recoverBus();
            reset();
            phase_ = Phase::RESET_WAIT;
            phaseMs_ = nowMs;
            return false;
        case Phase::RUN:
            break;
    }
    if (nowMs - lastReadMs_ < SENSOR_POLL_PERIOD_MS) return false;
    lastReadMs_ = nowMs;
    uint8_t buf[slf3s::FRAME_BYTES];
    const size_t got = wire_.requestFrom(static_cast<int>(slf3s::I2C_ADDRESS), static_cast<int>(slf3s::FRAME_BYTES));
    if (got != slf3s::FRAME_BYTES) {
        while (wire_.available()) wire_.read();
        fail(false, nowMs);
        return false;
    }
    for (uint8_t& b : buf) b = wire_.read();
    const slf3s::ParseResult r = slf3s::parseFrame(buf, sizeof(buf), s_);
    if (r != slf3s::ParseResult::OK) {
        fail(r == slf3s::ParseResult::CRC_ERROR, nowMs);
        return false;
    }
    err_.consecutive = 0;
    status_ = SensorStatus::OK;
    sampleMs_ = nowMs;
    return true;
}
