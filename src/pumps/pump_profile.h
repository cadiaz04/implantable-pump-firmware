#pragma once
// =============================================================================
// PumpProfile — a named pump with its motor settings, geometry, PID tuning
// and calibrations. Stored as JSON in LittleFS at /pumps/<id>.json.
// =============================================================================
#include <cmath>
#include <string>
#include <vector>
#include "calibration/curve_fit.h"
#include "motors/motor_types.h"

struct CalPointRec {
    float command = 0;
    float flow = NAN;       // sensor mean, µL/min
    float ref = NAN;        // manual reference, µL/min
    float std = NAN;
    float tempC = NAN;
    uint32_t n = 0;
    uint8_t pass = 0;
    std::string status;
};

struct Calibration {
    std::string id;
    std::string name;
    uint32_t timestamp = 0;     // epoch seconds (0 if the clock was never synced)
    std::string fluid = "water";
    std::string tubingNote;
    std::vector<CalPointRec> points;
    CurveFit fit;
    bool active = false;
};

struct PidParams {
    float kp = 0.3f;
    float ki = 0.05f;
    float kd = 0.0f;
    float loopHz = 1.0f;
    float filterWindowS = 10.0f;     // flow averaging window (control + display)
    bool filterEma = false;          // false: moving average, true: EMA (τ = window/2)
    float outputLimitPct = 100.0f;
    float integralLimitPct = 100.0f;
};

struct PumpProfile {
    std::string id;
    std::string name;
    std::string pumpType = "syringe";   // syringe | diaphragm | peristaltic | other
    std::string notes;
    MotorParams motor;
    float syringeIdMm = 0;              // 0 = not a syringe / unknown
    float leadscrewPitchMm = 0;
    std::string fluid = "water";        // water | saline
    PidParams pid;
    std::vector<Calibration> calibrations;

    const Calibration* activeCalibration() const {
        for (const auto& c : calibrations)
            if (c.active) return &c;
        return nullptr;
    }
    // µL/min per RPM from syringe geometry (steppers only), 0 if unknown.
    float theoreticalGain() const {
        if (familyOf(motor.driverType) != DriverFamily::STEPPER) return 0;
        return syringeUlMinPerRpm(syringeIdMm, leadscrewPitchMm);
    }
};

// Starter templates (uncalibrated).
PumpProfile starterProfile28byj();
PumpProfile starterProfileNema17();
PumpProfile starterProfileDcDiaphragm();
