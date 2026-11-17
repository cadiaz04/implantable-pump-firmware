#include <unity.h>
#include <cmath>
#include "../support/harness.h"

void setUp() {}
void tearDown() {}

static SweepConfig quick(int n, float lo, float hi) {
    SweepConfig c;
    c.nCommands = linearCommands(lo, hi, n, c.commands);
    c.settleS = 10;
    c.averageS = 10;
    c.zeroFirst = false;
    c.steadyWindowS = 5;
    c.steadyTimeoutS = 30;
    return c;
}

static void runToEnd(Harness& h) {
    int guard = 0;
    while (h.rm.state() == ChannelState::CALIBRATING && guard++ < 500000) h.tick();
}

void test_linear_commands() {
    float c[MAX_SWEEP_COMMANDS];
    TEST_ASSERT_EQUAL(5, linearCommands(0, 10, 5, c));
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 2.5f, c[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 10.0f, c[4]);
}

void test_rejects_bad_configs() {
    Harness h;
    SweepConfig c = quick(3, 0, 10);
    c.nCommands = 0;
    TEST_ASSERT_NOT_NULL(h.rm.startCalibration(c, true, 0));
    c = quick(20, 0, 10);
    c.repeats = 2;
    c.upDown = true;   // 80 points > 40
    TEST_ASSERT_NOT_NULL(h.rm.startCalibration(c, true, 0));
    c = quick(3, 0, 20);   // above maxCommand 15
    TEST_ASSERT_NOT_NULL(h.rm.startCalibration(c, true, 0));
}

void test_up_down_pass_order() {
    Harness h;
    SweepConfig c = quick(3, 0, 10);
    c.upDown = true;
    TEST_ASSERT_NULL(h.rm.startCalibration(c, true, 0));
    const CalibrationSweep& s = h.rm.sweep();
    TEST_ASSERT_EQUAL(6, s.pointCount());
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 10.0f, s.point(2).command);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 10.0f, s.point(3).command);
    TEST_ASSERT_EQUAL(1, s.point(3).pass);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, s.point(5).command);
}

void test_air_invalidates_point_then_retry() {
    Harness h;
    h.zeroOffset = 8;
    TEST_ASSERT_NULL(h.rm.startCalibration(quick(3, 0, 10), true, 0));
    // Run into the averaging phase of point 2 and inject a bubble there.
    while (!(h.rm.sweep().currentIndex() == 1 && h.rm.sweep().phase() == SweepPhase::AVERAGE)) h.tick();
    h.plant.injectBubble(1.0f);
    runToEnd(h);
    const CalibrationSweep& s = h.rm.sweep();
    TEST_ASSERT_EQUAL(static_cast<int>(SweepPhase::DONE), static_cast<int>(s.phase()));
    TEST_ASSERT_EQUAL(static_cast<int>(PointStatus::INVALID_AIR), static_cast<int>(s.point(1).status));
    TEST_ASSERT_EQUAL(2, s.validPointCount());
    TEST_ASSERT_EQUAL_FLOAT(0, h.out.command);   // motor stopped when done
    // Retry just that point.
    TEST_ASSERT_NULL(h.rm.retryCalibrationPoint(1, true, h.t));
    TEST_ASSERT_EQUAL(static_cast<int>(ChannelState::CALIBRATING), static_cast<int>(h.rm.state()));
    runToEnd(h);
    TEST_ASSERT_EQUAL(3, s.validPointCount());
    TEST_ASSERT_FLOAT_WITHIN(10.0, 100.0, s.point(1).flow);   // 20 µL/min/rpm × 5 rpm
}

void test_manual_reference_overrides_sensor() {
    Harness h;
    TEST_ASSERT_NULL(h.rm.startCalibration(quick(2, 5, 10), true, 0));
    TEST_ASSERT_FALSE(h.rm.sweep().setReference(0, 99));   // not measured yet
    runToEnd(h);
    CalibrationSweep& s = h.rm.sweep();
    TEST_ASSERT_TRUE(s.setReference(0, 123.0f));
    float x[4], y[4];
    TEST_ASSERT_EQUAL(2, s.collect(x, y, 4));
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 123.0f, y[0]);
    TEST_ASSERT_TRUE(s.clearReference(0));
    s.collect(x, y, 4);
    TEST_ASSERT_TRUE(std::fabs(y[0] - 123.0f) > 1);
}

void test_zero_phase_finds_offset_and_abort_stops() {
    Harness h;
    SweepConfig c = quick(3, 0, 10);
    c.zeroFirst = true;
    c.zeroSettleS = 2;
    c.zeroS = 10;
    TEST_ASSERT_NULL(h.rm.startCalibration(c, true, 0));
    while (h.rm.sweep().phase() != SweepPhase::SETTLE) h.tick();
    TEST_ASSERT_FLOAT_WITHIN(1.5, 8.0, h.zeroOffset);
    h.run(3);
    TEST_ASSERT_TRUE(h.out.command == 0.0f);   // point 1 is command 0
    h.rm.stop(h.t);
    h.run(0.1f);
    TEST_ASSERT_EQUAL(static_cast<int>(ChannelState::IDLE), static_cast<int>(h.rm.state()));
    TEST_ASSERT_EQUAL(static_cast<int>(SweepPhase::ABORTED), static_cast<int>(h.rm.sweep().phase()));
    TEST_ASSERT_EQUAL_FLOAT(0, h.out.command);
}

void test_sensor_fault_aborts_sweep() {
    Harness h;
    TEST_ASSERT_NULL(h.rm.startCalibration(quick(3, 0, 10), true, 0));
    h.run(5);
    h.sensorOk = false;
    h.run(0.1f);
    TEST_ASSERT_EQUAL(static_cast<int>(ChannelState::IDLE), static_cast<int>(h.rm.state()));
    TEST_ASSERT_TRUE(h.rm.alarms() & ALARM_SENSOR_FAULT);
}

void test_watchdog_respects_keep_running_for_calibration() {
    Harness h;
    h.hbExpired = true;
    TEST_ASSERT_NULL(h.rm.startCalibration(quick(2, 0, 10), false, 0));
    h.run(0.1f);
    TEST_ASSERT_EQUAL(static_cast<int>(ChannelState::IDLE), static_cast<int>(h.rm.state()));
    TEST_ASSERT_NULL(h.rm.startCalibration(quick(2, 0, 10), true, h.t));
    h.run(5);
    TEST_ASSERT_EQUAL(static_cast<int>(ChannelState::CALIBRATING), static_cast<int>(h.rm.state()));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_linear_commands);
    RUN_TEST(test_rejects_bad_configs);
    RUN_TEST(test_up_down_pass_order);
    RUN_TEST(test_air_invalidates_point_then_retry);
    RUN_TEST(test_manual_reference_overrides_sensor);
    RUN_TEST(test_zero_phase_finds_offset_and_abort_stops);
    RUN_TEST(test_sensor_fault_aborts_sweep);
    RUN_TEST(test_watchdog_respects_keep_running_for_calibration);
    return UNITY_END();
}
