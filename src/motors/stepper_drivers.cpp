#include "stepper_drivers.h"
#include <Arduino.h>
#include "motors/step_engine.h"

// ---------------- ULN2003 ----------------

StepperULN2003::StepperULN2003(int slot, const int pins[4]) : MotorDriver(DriverType::ULN2003), slot_(slot) {
    for (int i = 0; i < 4; ++i) pins_[i] = pins[i];
}

bool StepperULN2003::begin() {
    attached_ = step_engine::attach(slot_, step_engine::Mode::ULN4, pins_);
    return attached_;
}

void StepperULN2003::end() {
    if (attached_) step_engine::detach(slot_);
    attached_ = false;
    for (int p : pins_) {
        if (p >= 0) pinMode(p, INPUT);
    }
}

void StepperULN2003::apply(float rpm, bool reverse, float) {
    if (!attached_) return;
    const bool rev = reverse != params_.invertDir;
    sps_ = rpm > 0 ? rpmToStepsPerSec(rpm, params_) : 0;
    step_engine::setRate(slot_, sps_, rev);
    if (sps_ <= 0 && params_.releaseWhenStopped) step_engine::release(slot_);
}

void StepperULN2003::applyEnable(bool en) {
    if (!en && attached_) {
        step_engine::setRate(slot_, 0, false);
        step_engine::release(slot_);
    }
}

// ---------------- STEP/DIR ----------------

StepperStepDir::StepperStepDir(int slot, int stepPin, int dirPin, int enPin, bool enActiveLow)
    : MotorDriver(DriverType::STEP_DIR), slot_(slot), step_(stepPin), dir_(dirPin), en_(enPin), enActiveLow_(enActiveLow) {}

void StepperStepDir::writeEn(bool on) {
    if (on == energized_ && en_ >= 0 && attached_) return;
    energized_ = on;
    if (en_ < 0) return;
    digitalWrite(en_, (on != enActiveLow_) ? HIGH : LOW);
}

bool StepperStepDir::begin() {
    if (en_ >= 0) {
        pinMode(en_, OUTPUT);
        writeEn(false);   // boot with driver disabled
    }
    const int pins[2] = {step_, dir_};
    attached_ = step_engine::attach(slot_, step_engine::Mode::STEP_DIR, pins);
    return attached_;
}

void StepperStepDir::end() {
    if (attached_) step_engine::detach(slot_);
    attached_ = false;
    writeEn(false);
    pinMode(step_, INPUT);
    pinMode(dir_, INPUT);
    // Leave EN driven to its disabled level; a floating EN would enable most
    // drivers (they pull EN low internally).
}

void StepperStepDir::apply(float rpm, bool reverse, float) {
    if (!attached_) return;
    const bool rev = reverse != params_.invertDir;
    sps_ = rpm > 0 ? rpmToStepsPerSec(rpm, params_) : 0;
    // Energise before the first step; de-energise when stopped if configured.
    writeEn(enabled_ && (sps_ > 0 || !params_.releaseWhenStopped));
    step_engine::setRate(slot_, enabled_ ? sps_ : 0, rev);
}

void StepperStepDir::applyEnable(bool en) {
    if (!en) {
        if (attached_) step_engine::setRate(slot_, 0, false);
        writeEn(false);
    }
}
