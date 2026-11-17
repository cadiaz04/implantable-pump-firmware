#pragma once
// =============================================================================
// Brushed DC motor drivers (no encoder: command is PWM duty %).
//
// DcDrv8871 — TI DRV8871, IN1/IN2 both PWM (datasheet Table "H-Bridge
//             Control"):   IN1 IN2  → OUT
//                           0   0   coast (sleep after ~1 ms)
//                           0   1   reverse
//                           1   0   forward
//                           1   1   brake (slow decay)
//   PWM, fast decay: forward IN1 = PWM(d), IN2 = 0;  reverse IN1 = 0, IN2 = PWM(d)
//   PWM, slow decay: forward IN1 = 1, IN2 = PWM(1−d); reverse IN1 = PWM(1−d), IN2 = 1
//
// DcPwmDir  — generic PWM + DIR (+ optional EN/STBY) boards: L298N, TB6612.
//
// Both: LEDC at 20 kHz (inaudible), 10-bit. Deadband (minDuty): 0 < d <
// minDuty drives at 0. Optional start kick (kickDuty for kickMs) to break
// stiction when starting from rest. Hook for a future tach/encoder input:
// see DcMotorBase::tachPulse().
// =============================================================================
#include "motors/motor_driver.h"

class DcMotorBase : public MotorDriver {
public:
    DcMotorBase(DriverType t, int slot) : MotorDriver(t), slot_(slot) {}
    // Future encoder/tach hook: call from a pulse-counter ISR / PCNT poll.
    void tachPulse(uint32_t count) { tachCount_ += count; }
    uint32_t tachCount() const { return tachCount_; }

protected:
    void apply(float duty, bool reverse, float dtS) override;
    virtual void writeDuty(float d01, bool forward) = 0;   // d01 in [0,1], 0 = coast
    int ledcChannel(int k) const { return slot_ * 2 + k; }
    void setupLedc(int pin, int k);
    void writeLedc(int k, float d01);

    int slot_;
    float kickRemainingS_ = 0;
    bool wasStopped_ = true;
    volatile uint32_t tachCount_ = 0;
};

class DcDrv8871 : public DcMotorBase {
public:
    DcDrv8871(int slot, int in1, int in2);
    bool begin() override;
    void end() override;

protected:
    void writeDuty(float d01, bool forward) override;
    void applyEnable(bool en) override;

private:
    int in1_, in2_;
};

class DcPwmDir : public DcMotorBase {
public:
    DcPwmDir(int slot, int pwm, int dir, int en);
    bool begin() override;
    void end() override;

protected:
    void writeDuty(float d01, bool forward) override;
    void applyEnable(bool en) override;

private:
    int pwm_, dir_, en_;
};
