#include "motor_types.h"
#include <cmath>
#include <cstring>

const char* driverTypeId(DriverType t) {
    switch (t) {
        case DriverType::ULN2003: return "uln2003";
        case DriverType::STEP_DIR: return "stepdir";
        case DriverType::DRV8871: return "drv8871";
        case DriverType::PWM_DIR: return "pwmdir";
        default: return "none";
    }
}

DriverType driverTypeFromId(const char* id) {
    if (!id) return DriverType::NONE;
    if (!strcmp(id, "uln2003")) return DriverType::ULN2003;
    if (!strcmp(id, "stepdir")) return DriverType::STEP_DIR;
    if (!strcmp(id, "drv8871")) return DriverType::DRV8871;
    if (!strcmp(id, "pwmdir")) return DriverType::PWM_DIR;
    return DriverType::NONE;
}

const char* commandUnitFor(DriverType t) {
    return familyOf(t) == DriverFamily::DC ? "Duty %" : "RPM";
}

int requiredPinCount(DriverType t) {
    switch (t) {
        case DriverType::ULN2003: return 4;
        case DriverType::STEP_DIR: return 2;   // STEP, DIR (+ optional EN)
        case DriverType::DRV8871: return 2;
        case DriverType::PWM_DIR: return 2;    // PWM, DIR (+ optional EN/STBY)
        default: return 0;
    }
}

int maxPinCount(DriverType t) {
    switch (t) {
        case DriverType::ULN2003: return 4;
        case DriverType::STEP_DIR: return 3;
        case DriverType::DRV8871: return 2;
        case DriverType::PWM_DIR: return 3;
        default: return 0;
    }
}

float rpmToStepsPerSec(float rpm, const MotorParams& m) {
    return rpm * m.stepsPerRev * m.microstep * m.gearRatio / 60.0f;
}

float stepsPerSecToRpm(float sps, const MotorParams& m) {
    const float k = m.stepsPerRev * m.microstep * m.gearRatio;
    return k > 0 ? sps * 60.0f / k : 0.0f;
}

float syringeUlMinPerRpm(float innerDiameterMm, float leadscrewPitchMm) {
    if (!(innerDiameterMm > 0) || !(leadscrewPitchMm > 0)) return 0.0f;
    const float area = 3.14159265f * 0.25f * innerDiameterMm * innerDiameterMm;
    return area * leadscrewPitchMm;
}

float rampToward(float current, float target, float accelPerS, float dtS) {
    if (!(accelPerS > 0)) return target;   // accel 0 = no ramp limiting
    const float maxDelta = accelPerS * dtS;
    const float d = target - current;
    if (std::fabs(d) <= maxDelta) return target;
    return current + (d > 0 ? maxDelta : -maxDelta);
}
