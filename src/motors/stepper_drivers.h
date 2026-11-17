#pragma once
// Stepper drivers built on the timer-interrupt step engine.
//   StepperULN2003 — unipolar 28BYJ-48 via ULN2003 board (IN1..IN4, half-step)
//   StepperStepDir — A4988 / DRV8825 / TMC2209 (STEP, DIR, optional EN)
#include "motors/motor_driver.h"

class StepperULN2003 : public MotorDriver {
public:
    StepperULN2003(int slot, const int pins[4]);
    bool begin() override;
    void end() override;
    float stepRateHz() const override { return sps_; }

protected:
    void apply(float rpm, bool reverse, float dtS) override;
    void applyEnable(bool en) override;

private:
    int slot_;
    int pins_[4];
    float sps_ = 0;
    bool attached_ = false;
};

class StepperStepDir : public MotorDriver {
public:
    StepperStepDir(int slot, int stepPin, int dirPin, int enPin, bool enActiveLow);
    bool begin() override;
    void end() override;
    float stepRateHz() const override { return sps_; }

protected:
    void apply(float rpm, bool reverse, float dtS) override;
    void applyEnable(bool en) override;

private:
    void writeEn(bool on);
    int slot_;
    int step_, dir_, en_;
    bool enActiveLow_;
    float sps_ = 0;
    bool attached_ = false;
    bool energized_ = false;
};
