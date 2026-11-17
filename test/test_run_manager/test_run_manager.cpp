#include <unity.h>
#include <cmath>
#include "../support/harness.h"

void setUp() {}
void tearDown() {}

static RunRequest manual(float cmd, bool keep = false) {
    RunRequest r;
    r.mode = RunMode::MANUAL;
    r.command = cmd;
    r.keepRunning = keep;
    return r;
}

void test_manual_limits() {
    Harness h(15, 2);
    TEST_ASSERT_NOT_NULL(h.rm.start(manual(16), 0, true, true));     // > max
    TEST_ASSERT_NOT_NULL(h.rm.start(manual(1), 0, true, true));      // < min
    TEST_ASSERT_NOT_NULL(h.rm.start(manual(NAN), 0, true, true));
    TEST_ASSERT_NOT_NULL(h.rm.start(manual(-1), 0, true, true));
    TEST_ASSERT_NULL(h.rm.start(manual(5), 0, true, true));
    h.run(0.1f);
    TEST_ASSERT_EQUAL(static_cast<int>(ChannelState::RUNNING), static_cast<int>(h.rm.state()));
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 5.0f, h.out.command);
    // live change
    TEST_ASSERT_NULL(h.rm.start(manual(7), 0, true, true));
    h.run(0.1f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 7.0f, h.out.command);
    h.rm.stop(h.t);
    h.run(0.1f);
    TEST_ASSERT_EQUAL_FLOAT(0, h.out.command);
    TEST_ASSERT_EQUAL(static_cast<int>(ChannelState::IDLE), static_cast<int>(h.rm.state()));
}

void test_closed_loop_rejects_bad_setpoints() {
    Harness h;
    RunRequest r;
    r.mode = RunMode::CLOSED_LOOP;
    r.initialCommand = 3;
    r.flowUlMin = NAN;
    TEST_ASSERT_NOT_NULL(h.rm.start(r, 0, true, true));
    r.flowUlMin = MAX_FLOW_UL_MIN + 1;
    TEST_ASSERT_NOT_NULL(h.rm.start(r, 0, true, true));
    r.flowUlMin = -5;
    TEST_ASSERT_NOT_NULL(h.rm.start(r, 0, true, true));
    r.flowUlMin = 62.5f;
    TEST_ASSERT_NOT_NULL(h.rm.start(r, 0, false, false));   // no sensor
    TEST_ASSERT_NOT_NULL(h.rm.start(r, 0, true, false));    // sensor fault
    r.initialCommand = 0;
    TEST_ASSERT_NOT_NULL(h.rm.start(r, 0, true, true));     // no model at all
    r.initialCommand = 3;
    TEST_ASSERT_NULL(h.rm.start(r, 0, true, true));
    TEST_ASSERT_EQUAL(static_cast<int>(ChannelState::IDLE), static_cast<int>(ChannelState::IDLE));
}

void test_watchdog_stops_run() {
    Harness h;
    TEST_ASSERT_NULL(h.rm.start(manual(5), 0, true, true));
    h.run(1);
    h.hbExpired = true;
    h.run(0.05f);
    TEST_ASSERT_EQUAL(static_cast<int>(ChannelState::IDLE), static_cast<int>(h.rm.state()));
    TEST_ASSERT_TRUE(h.rm.alarms() & ALARM_WATCHDOG);
    TEST_ASSERT_EQUAL_FLOAT(0, h.out.command);
}

void test_keep_running_ignores_watchdog() {
    Harness h;
    TEST_ASSERT_NULL(h.rm.start(manual(5, true), 0, true, true));
    h.hbExpired = true;
    h.run(5);
    TEST_ASSERT_EQUAL(static_cast<int>(ChannelState::RUNNING), static_cast<int>(h.rm.state()));
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 5.0f, h.out.command);
}

void test_estop_latches() {
    Harness h;
    TEST_ASSERT_NULL(h.rm.start(manual(5, true), 0, true, true));
    h.run(0.5f);
    h.rm.estop();
    h.run(0.1f);
    TEST_ASSERT_EQUAL(static_cast<int>(ChannelState::ESTOP), static_cast<int>(h.rm.state()));
    TEST_ASSERT_EQUAL_FLOAT(0, h.out.command);
    TEST_ASSERT_NOT_NULL(h.rm.start(manual(5), h.t, true, true));
    h.rm.stop(h.t);   // stop must not clear an E-STOP
    TEST_ASSERT_EQUAL(static_cast<int>(ChannelState::ESTOP), static_cast<int>(h.rm.state()));
    h.rm.resetEstop();
    TEST_ASSERT_EQUAL(static_cast<int>(ChannelState::IDLE), static_cast<int>(h.rm.state()));
    h.run(0.1f);
    TEST_ASSERT_EQUAL_FLOAT(0, h.out.command);   // nothing auto-resumes
    TEST_ASSERT_NULL(h.rm.start(manual(5), h.t, true, true));
}

void test_sensor_fault_pauses_closed_loop() {
    Harness h;
    RunRequest r;
    r.mode = RunMode::CLOSED_LOOP;
    r.flowUlMin = 60;
    r.initialCommand = 3;
    r.keepRunning = true;
    TEST_ASSERT_NULL(h.rm.start(r, 0, true, true));
    h.run(5);
    h.sensorOk = false;
    h.run(0.1f);
    TEST_ASSERT_EQUAL(static_cast<int>(ChannelState::PAUSED), static_cast<int>(h.rm.state()));
    TEST_ASSERT_EQUAL_FLOAT(0, h.out.command);
    TEST_ASSERT_TRUE(h.rm.alarms() & ALARM_SENSOR_FAULT);
    TEST_ASSERT_NOT_NULL(h.rm.resume(h.t, false, false));
    h.sensorOk = true;
    h.run(0.1f);
    TEST_ASSERT_NULL(h.rm.resume(h.t, true, false));
    h.run(0.1f);
    TEST_ASSERT_EQUAL(static_cast<int>(ChannelState::RUNNING), static_cast<int>(h.rm.state()));
}

void test_air_in_line_pauses_feedback_but_not_manual() {
    Harness h;
    TEST_ASSERT_NULL(h.rm.start(manual(5, true), 0, true, true));
    h.forceAir = true;
    h.run(2);
    TEST_ASSERT_EQUAL(static_cast<int>(ChannelState::RUNNING), static_cast<int>(h.rm.state()));   // priming
    TEST_ASSERT_TRUE(h.rm.alarms() & ALARM_AIR);
    h.forceAir = false;
    h.rm.stop(h.t);
    RunRequest r;
    r.mode = RunMode::CLOSED_LOOP;
    r.flowUlMin = 60;
    r.initialCommand = 3;
    r.keepRunning = true;
    TEST_ASSERT_NULL(h.rm.start(r, h.t, true, true));
    h.run(2);
    h.forceAir = true;
    h.run(0.3f);   // below debounce
    TEST_ASSERT_EQUAL(static_cast<int>(ChannelState::RUNNING), static_cast<int>(h.rm.state()));
    h.run(0.5f);
    TEST_ASSERT_EQUAL(static_cast<int>(ChannelState::PAUSED), static_cast<int>(h.rm.state()));
}

void test_flow_ceiling_trips_fault_in_manual() {
    Harness h(100);
    TEST_ASSERT_NULL(h.rm.start(manual(60, true), 0, true, true));   // 20·60 = 1200 µL/min > ceiling
    h.run(30);
    TEST_ASSERT_EQUAL(static_cast<int>(ChannelState::FAULT), static_cast<int>(h.rm.state()));
    TEST_ASSERT_TRUE(h.rm.alarms() & ALARM_FLOW_CEILING);
    TEST_ASSERT_EQUAL_FLOAT(0, h.out.command);
    TEST_ASSERT_NOT_NULL(h.rm.start(manual(5), h.t, true, true));
    h.rm.stop(h.t);   // acknowledge
    TEST_ASSERT_EQUAL(static_cast<int>(ChannelState::IDLE), static_cast<int>(h.rm.state()));
}

void test_volume_mode_completes() {
    Harness h;
    h.zeroOffset = 8;   // pretend tare already done
    RunRequest r;
    r.mode = RunMode::VOLUME;
    r.volumeUl = 50;
    r.durationS = 30;   // → 100 µL/min
    r.initialCommand = 5;
    r.keepRunning = true;
    TEST_ASSERT_NULL(h.rm.start(r, 0, true, true));
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 100.0f, h.rm.flowSetpoint());
    h.run(120);
    TEST_ASSERT_EQUAL(static_cast<int>(ChannelState::IDLE), static_cast<int>(h.rm.state()));
    TEST_ASSERT_FLOAT_WITHIN(3.0, 50.0, h.rm.runVolumeUl());
    TEST_ASSERT_TRUE(h.rm.volumeUl() >= 50.0);
}

void test_volume_mode_rejects_excess_rate() {
    Harness h;
    RunRequest r;
    r.mode = RunMode::VOLUME;
    r.volumeUl = 15000;
    r.durationS = 60;   // 15 mL/min ≫ ceiling
    r.initialCommand = 3;
    TEST_ASSERT_NOT_NULL(h.rm.start(r, 0, true, true));
}

void test_program_steps_and_completion() {
    Harness h;
    RunRequest r;
    r.mode = RunMode::PROGRAM;
    r.keepRunning = true;
    r.program.n = 2;
    r.program.steps[0] = {StepKind::MANUAL, 3, 2};
    r.program.steps[1] = {StepKind::MANUAL, 6, 2};
    TEST_ASSERT_NULL(h.rm.start(r, 0, true, true));
    h.run(1);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 3.0f, h.out.command);
    h.run(2);
    TEST_ASSERT_EQUAL(1, h.rm.programStep());
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 6.0f, h.out.command);
    h.run(2);
    TEST_ASSERT_EQUAL(static_cast<int>(ChannelState::IDLE), static_cast<int>(h.rm.state()));
}

void test_zero_tare() {
    Harness h;
    TEST_ASSERT_NULL(h.rm.startZero(5, 0));
    TEST_ASSERT_NOT_NULL(h.rm.start(manual(5), 0, true, true));   // busy
    h.run(6);
    TEST_ASSERT_FALSE(h.rm.zeroing());
    TEST_ASSERT_FLOAT_WITHIN(1.0, 8.0, h.zeroOffset);
}

void test_open_loop_needs_model() {
    Harness h;
    RunRequest r;
    r.mode = RunMode::OPEN_LOOP;
    r.flowUlMin = 50;
    TEST_ASSERT_NOT_NULL(h.rm.start(r, 0, true, true));
    RunConfig c = h.rm.config();
    c.theoreticalGain = 20;
    h.rm.configure(c);
    TEST_ASSERT_NULL(h.rm.start(r, 0, true, true));
    h.run(0.1f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 2.5f, h.out.command);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_manual_limits);
    RUN_TEST(test_closed_loop_rejects_bad_setpoints);
    RUN_TEST(test_watchdog_stops_run);
    RUN_TEST(test_keep_running_ignores_watchdog);
    RUN_TEST(test_estop_latches);
    RUN_TEST(test_sensor_fault_pauses_closed_loop);
    RUN_TEST(test_air_in_line_pauses_feedback_but_not_manual);
    RUN_TEST(test_flow_ceiling_trips_fault_in_manual);
    RUN_TEST(test_volume_mode_completes);
    RUN_TEST(test_volume_mode_rejects_excess_rate);
    RUN_TEST(test_program_steps_and_completion);
    RUN_TEST(test_zero_tare);
    RUN_TEST(test_open_loop_needs_model);
    return UNITY_END();
}
