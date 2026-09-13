#pragma once
// =============================================================================
// flow_controller.h — Converts a desired flow rate (mL/hr) into a step
// pulse frequency (Hz) for the stepper driver.
//
// DELIBERATELY has zero Arduino/ESP32 dependencies. This is pure math so it
// can be unit tested on your laptop (pio test -e native) without any
// hardware attached — important for a safety-relevant calculation like this.
// =============================================================================

#include <cstdint>

struct PumpGeometry {
    float stepsPerRevolution;   // motor full steps/rev
    float microstepping;        // driver microstepping factor
    float leadScrewMmPerRev;    // linear travel per revolution
    float syringeAreaMm2;       // syringe barrel cross-sectional area
};

class FlowController {
public:
    FlowController(const PumpGeometry& geometry, float minFlowMlHr, float maxFlowMlHr);

    // Returns the step frequency (Hz) needed to achieve targetFlowMlHr.
    // Clamps to [minFlowMlHr, maxFlowMlHr] and returns 0 Hz if the input is
    // invalid (NaN, negative-below-min, etc.) — callers must treat 0 Hz +
    // isSafe()==false as "do not move."
    float computeStepFrequencyHz(float targetFlowMlHr);

    // True if the last computeStepFrequencyHz() call was within safe bounds.
    bool isSafe() const { return lastCommandWasSafe_; }

    // Clamps a requested flow rate into the allowed range; does not change state.
    float clampFlowRate(float requestedMlHr) const;

private:
    PumpGeometry geometry_;
    float minFlowMlHr_;
    float maxFlowMlHr_;
    bool lastCommandWasSafe_ = true;

    // mL/hr -> steps/sec, derived from pump geometry.
    float mlPerHourToHz(float mlHr) const;
};
