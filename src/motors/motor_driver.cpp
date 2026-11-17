#include "motor_driver.h"
#include <cmath>
#include "config.h"

float MotorDriver::maxAllowed() const {
    const float abs = familyOf(type_) == DriverFamily::DC ? ABS_MAX_DUTY_PCT : ABS_MAX_STEPPER_RPM;
    float m = params_.maxCommand;
    if (!(m > 0)) return 0;
    if (m > abs) m = abs;
    if (familyOf(type_) == DriverFamily::STEPPER) {
        // Also respect the step generator's rate limit.
        const float sps = rpmToStepsPerSec(m, params_);
        if (sps > ABS_MAX_STEP_RATE_HZ) m = stepsPerSecToRpm(ABS_MAX_STEP_RATE_HZ, params_);
    }
    return m;
}

void MotorDriver::setTarget(float command, bool reverse) {
    if (!std::isfinite(command) || command < 0) command = 0;
    const float mx = maxAllowed();
    if (command > mx) command = mx;
    target_ = command;
    reverse_ = reverse;
}

void MotorDriver::stop() {
    target_ = 0;
    current_ = 0;
    apply(0, curReverse_, 0);
}

void MotorDriver::enable(bool en) {
    enabled_ = en;
    if (!en) stop();
    applyEnable(en);
}

void MotorDriver::update(float dtS) {
    if (!enabled_) {
        current_ = target_ = 0;
        apply(0, curReverse_, dtS);
        return;
    }
    float goal = target_;
    if (reverse_ != curReverse_) {
        // Ramp down to zero before reversing.
        goal = 0;
        if (current_ <= 0) curReverse_ = reverse_;
    }
    current_ = rampToward(current_, goal, params_.accel, dtS);
    if (current_ < 0) current_ = 0;
    apply(current_, curReverse_, dtS);
}
