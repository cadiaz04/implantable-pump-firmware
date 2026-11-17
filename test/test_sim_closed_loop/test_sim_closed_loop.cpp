// End-to-end (host-side): calibrate a simulated pump with the firmware sweep,
// fit it, then hold 62.5 µL/min in closed loop — the user's main use case.
#include <unity.h>
#include <cmath>
#include "../support/harness.h"
#include "calibration/curve_fit.h"
#include "control/flow_model.h"

void setUp() {}
void tearDown() {}

static CurveFit calibrate(Harness& h, float lo, float hi, int n, FitType type) {
    SweepConfig sc;
    sc.nCommands = linearCommands(lo, hi, n, sc.commands);
    sc.settleS = 15;
    sc.averageS = 30;
    sc.zeroFirst = true;
    sc.zeroSettleS = 10;
    sc.zeroS = 20;
    TEST_ASSERT_NULL(h.rm.startCalibration(sc, true, h.t));
    int guard = 0;
    while (h.rm.state() == ChannelState::CALIBRATING && guard++ < 200000) h.tick();
    TEST_ASSERT_EQUAL(static_cast<int>(SweepPhase::DONE), static_cast<int>(h.rm.sweep().phase()));
    TEST_ASSERT_EQUAL(n, h.rm.sweep().validPointCount());
    float x[MAX_SWEEP_POINTS], y[MAX_SWEEP_POINTS];
    const int k = h.rm.sweep().collect(x, y, MAX_SWEEP_POINTS);
    CurveFit f;
    const char* err = nullptr;
    TEST_ASSERT_TRUE_MESSAGE(fitCurve(x, y, k, type, f, &err), err);
    return f;
}

static float holdAndMeasure(Harness& h, float target, float seconds, float measureLastS) {
    RunRequest r;
    r.mode = RunMode::CLOSED_LOOP;
    r.flowUlMin = target;
    r.keepRunning = true;
    TEST_ASSERT_NULL(h.rm.start(r, h.t, true, true));
    h.run(seconds - measureLastS);
    double sum = 0;
    long n = 0;
    const int ticks = static_cast<int>(measureLastS / 0.01f);
    for (int i = 0; i < ticks; ++i) {
        h.tick();
        sum += h.plant.trueFlow();
        ++n;
    }
    TEST_ASSERT_EQUAL(static_cast<int>(ChannelState::RUNNING), static_cast<int>(h.rm.state()));
    return static_cast<float>(sum / n);
}

void test_stepper_calibrate_then_hold_62_5() {
    Harness h(15);
    CurveFit f = calibrate(h, 0, 10, 6, FitType::LINEAR);
    TEST_ASSERT_FLOAT_WITHIN(1.0, 20.0, f.c[1]);   // plant gain 20 µL/min per rpm
    TEST_ASSERT_TRUE(f.r2 > 0.995f);
    TEST_ASSERT_FLOAT_WITHIN(1.5, 8.0, h.zeroOffset);   // tare found the sensor offset
    CurveFlowModel model(f);
    h.rm.setCalibrationModel(&model);
    const float flow = holdAndMeasure(h, 62.5f, 400, 150);
    TEST_ASSERT_FLOAT_WITHIN(62.5f * 0.03f, 62.5f, flow);
}

void test_pi_corrects_calibration_error() {
    Harness h(15);
    CurveFit f = calibrate(h, 0, 10, 5, FitType::LINEAR);
    CurveFlowModel model(f);
    h.rm.setCalibrationModel(&model);
    // Pump drifts 20% weaker after calibration (e.g. different tubing).
    SimPlantParams p = h.plant.params();
    p.gain *= 0.8f;
    h.plant.setParams(p);
    const float flow = holdAndMeasure(h, 62.5f, 600, 200);
    TEST_ASSERT_FLOAT_WITHIN(62.5f * 0.03f, 62.5f, flow);
}

void test_dc_pump_quadratic_with_deadband_and_pulsation() {
    Harness h(100, 15);
    SimPlantParams p;
    p.gain = 6;
    p.quad = 0.02f;
    p.deadband = 12;
    p.pulsationPct = 25;
    p.pulsationHzPerCmd = 0.04f;
    p.noiseUlMin = 5;
    p.zeroOffsetUlMin = -5;
    p.tauS = 3;
    p.deadTimeS = 2;
    h.plant.setParams(p);
    h.avg.configure(20);   // pulsatile → longer window
    CurveFit f = calibrate(h, 15, 100, 8, FitType::QUADRATIC);
    TEST_ASSERT_TRUE(f.r2 > 0.99f);
    CurveFlowModel model(f);
    h.rm.setCalibrationModel(&model);
    const float flow = holdAndMeasure(h, 62.5f, 500, 200);
    TEST_ASSERT_FLOAT_WITHIN(62.5f * 0.03f, 62.5f, flow);
}

void test_closed_loop_without_calibration_uses_initial_command() {
    Harness h(15);
    h.zeroOffset = 8;
    RunRequest r;
    r.mode = RunMode::CLOSED_LOOP;
    r.flowUlMin = 62.5f;
    r.initialCommand = 2.0f;   // user's guess (true answer 3.125)
    r.keepRunning = true;
    TEST_ASSERT_NULL(h.rm.start(r, h.t, true, true));
    h.run(500);
    double s = 0;
    for (int i = 0; i < 15000; ++i) {
        h.tick();
        s += h.plant.trueFlow();
    }
    TEST_ASSERT_FLOAT_WITHIN(62.5f * 0.03f, 62.5f, static_cast<float>(s / 15000));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_stepper_calibrate_then_hold_62_5);
    RUN_TEST(test_pi_corrects_calibration_error);
    RUN_TEST(test_dc_pump_quadratic_with_deadband_and_pulsation);
    RUN_TEST(test_closed_loop_without_calibration_uses_initial_command);
    return UNITY_END();
}
