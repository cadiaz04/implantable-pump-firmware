#include "dc_hbridge.h"
#include <Arduino.h>

namespace {
constexpr uint32_t kPwmHz = 20000;
constexpr uint8_t kBits = 10;
constexpr uint32_t kMax = (1u << kBits) - 1;   // Arduino core maps max → 100 % on
}

void DcMotorBase::setupLedc(int pin, int k) {
    ledcSetup(ledcChannel(k), kPwmHz, kBits);
    ledcAttachPin(pin, ledcChannel(k));
    ledcWrite(ledcChannel(k), 0);
}

void DcMotorBase::writeLedc(int k, float d01) {
    if (d01 < 0) d01 = 0;
    if (d01 > 1) d01 = 1;
    ledcWrite(ledcChannel(k), static_cast<uint32_t>(d01 * kMax + 0.5f));
}

void DcMotorBase::apply(float duty, bool reverse, float dtS) {
    float d = duty;
    if (d > 0 && d < params_.minDuty) d = 0;   // deadband
    if (d <= 0) {
        wasStopped_ = true;
        kickRemainingS_ = 0;
        writeDuty(0, true);
        return;
    }
    if (wasStopped_) {
        wasStopped_ = false;
        kickRemainingS_ = params_.kickMs / 1000.0f;
    }
    if (kickRemainingS_ > 0) {
        kickRemainingS_ -= dtS;
        float kick = params_.kickDuty;
        if (kick > params_.maxCommand) kick = params_.maxCommand;
        if (kick > d) d = kick;
    }
    const bool forward = !(reverse != params_.invertDir);
    writeDuty(d / 100.0f, forward);
}

// ---------------- DRV8871 ----------------

DcDrv8871::DcDrv8871(int slot, int in1, int in2) : DcMotorBase(DriverType::DRV8871, slot), in1_(in1), in2_(in2) {}

bool DcDrv8871::begin() {
    setupLedc(in1_, 0);
    setupLedc(in2_, 1);
    return true;
}

void DcDrv8871::end() {
    writeLedc(0, 0);
    writeLedc(1, 0);
    ledcDetachPin(in1_);
    ledcDetachPin(in2_);
    pinMode(in1_, OUTPUT);
    digitalWrite(in1_, LOW);
    pinMode(in2_, OUTPUT);
    digitalWrite(in2_, LOW);
}

void DcDrv8871::writeDuty(float d, bool forward) {
    if (d <= 0) {   // coast
        writeLedc(0, 0);
        writeLedc(1, 0);
        return;
    }
    if (params_.decay == DecayMode::FAST) {
        writeLedc(0, forward ? d : 0);
        writeLedc(1, forward ? 0 : d);
    } else {
        writeLedc(0, forward ? 1 : 1 - d);
        writeLedc(1, forward ? 1 - d : 1);
    }
}

void DcDrv8871::applyEnable(bool en) {
    if (!en) writeDuty(0, true);
}

// ---------------- PWM + DIR ----------------

DcPwmDir::DcPwmDir(int slot, int pwm, int dir, int en) : DcMotorBase(DriverType::PWM_DIR, slot), pwm_(pwm), dir_(dir), en_(en) {}

bool DcPwmDir::begin() {
    setupLedc(pwm_, 0);
    pinMode(dir_, OUTPUT);
    digitalWrite(dir_, LOW);
    if (en_ >= 0) {
        pinMode(en_, OUTPUT);
        digitalWrite(en_, LOW);
    }
    return true;
}

void DcPwmDir::end() {
    writeLedc(0, 0);
    ledcDetachPin(pwm_);
    pinMode(pwm_, OUTPUT);
    digitalWrite(pwm_, LOW);
    pinMode(dir_, INPUT);
    if (en_ >= 0) digitalWrite(en_, LOW);
}

void DcPwmDir::writeDuty(float d, bool forward) {
    digitalWrite(dir_, forward ? LOW : HIGH);
    if (en_ >= 0) digitalWrite(en_, (d > 0 && enabled_) ? HIGH : LOW);
    writeLedc(0, d);
}

void DcPwmDir::applyEnable(bool en) {
    if (!en) writeDuty(0, true);
}
