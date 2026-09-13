#include "flow_controller.h"
#include <cmath>

FlowController::FlowController(const PumpGeometry& geometry, float minFlowMlHr, float maxFlowMlHr)
    : geometry_(geometry), minFlowMlHr_(minFlowMlHr), maxFlowMlHr_(maxFlowMlHr) {}

float FlowController::clampFlowRate(float requestedMlHr) const {
    if (requestedMlHr < minFlowMlHr_) return minFlowMlHr_;
    if (requestedMlHr > maxFlowMlHr_) return maxFlowMlHr_;
    return requestedMlHr;
}

float FlowController::mlPerHourToHz(float mlHr) const {
    // mL/hr -> mm^3/hr -> mm travel/hr -> rev/hr -> steps/hr -> steps/sec
    const float mm3PerHour = mlHr * 1000.0f;                              // 1 mL = 1000 mm^3
    const float mmTravelPerHour = mm3PerHour / geometry_.syringeAreaMm2;
    const float revPerHour = mmTravelPerHour / geometry_.leadScrewMmPerRev;
    const float stepsPerHour = revPerHour * geometry_.stepsPerRevolution * geometry_.microstepping;
    return stepsPerHour / 3600.0f;   // steps/sec
}

float FlowController::computeStepFrequencyHz(float targetFlowMlHr) {
    if (std::isnan(targetFlowMlHr) || std::isinf(targetFlowMlHr)) {
        lastCommandWasSafe_ = false;
        return 0.0f;
    }

    if (targetFlowMlHr < minFlowMlHr_ || targetFlowMlHr > maxFlowMlHr_) {
        lastCommandWasSafe_ = false;
        return 0.0f;   // reject out-of-range commands outright rather than silently clamping
    }

    lastCommandWasSafe_ = true;
    return mlPerHourToHz(targetFlowMlHr);
}
