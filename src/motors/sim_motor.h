#pragma once
// SIM_MODE motor: no GPIO at all, just the ramped command. The channel feeds
// currentCommand() into the simulated plant.
#include "motors/motor_driver.h"

class SimMotor : public MotorDriver {
public:
    explicit SimMotor(DriverType t) : MotorDriver(t) {}
    bool begin() override { return true; }
    void end() override {}
    float stepRateHz() const override {
        return familyOf(type_) == DriverFamily::STEPPER ? rpmToStepsPerSec(current_, params_) : 0;
    }

protected:
    void apply(float, bool, float) override {}
    void applyEnable(bool) override {}
};
