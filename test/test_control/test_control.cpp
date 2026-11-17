#include <unity.h>
#include <cmath>
#include "control/flow_controller.h"
#include "control/pid.h"
#include "control/watchdog.h"

void setUp() {}
void tearDown() {}

void test_pid_proportional_only() {
    Pid p;
    PidConfig c;
    c.kp = 2;
    p.configure(c);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 20.0f, p.update(10, 0, 1));
}

void test_pid_integral_clamped() {
    Pid p;
    PidConfig c;
    c.ki = 1;
    c.iMin = -5;
    c.iMax = 5;
    p.configure(c);
    for (int i = 0; i < 100; ++i) p.update(10, 0, 1);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 5.0f, p.integral());
}

void test_pid_conditional_integration_antiwindup() {
    Pid p;
    PidConfig c;
    c.kp = 1;
    c.ki = 1;
    c.outMin = -10;
    c.outMax = 10;
    p.configure(c);
    // Large error saturates the output: integrator must not wind up.
    for (int i = 0; i < 50; ++i) TEST_ASSERT_FLOAT_WITHIN(1e-6, 10.0f, p.update(100, 0, 1));
    TEST_ASSERT_TRUE(p.integral() <= 10.0f);
    // As soon as the error reverses the output must leave saturation promptly.
    const float out = p.update(0, 5, 1);
    TEST_ASSERT_TRUE(out < 10.0f);
}

void test_pid_actuator_saturation_freezes_integrator() {
    Pid p;
    PidConfig c;
    c.ki = 1;
    p.configure(c);
    p.update(1, 0, 1);
    const float i0 = p.integral();
    p.update(1, 0, 1, +1);   // actuator saturated high
    TEST_ASSERT_FLOAT_WITHIN(1e-6, i0, p.integral());
    p.update(-1, 0, 1, +1);  // moving away from saturation is allowed
    TEST_ASSERT_TRUE(p.integral() < i0);
}

void test_flow_controller_rejects_out_of_range() {
    FlowController fc;
    FlowControlConfig cfg;
    cfg.minCommand = 0;
    cfg.maxCommand = 10;
    fc.configure(cfg);
    ProportionalFlowModel m(25);
    fc.setModel(&m);
    TEST_ASSERT_FALSE(fc.setSetpoint(NAN));
    TEST_ASSERT_FALSE(fc.isSafe());
    TEST_ASSERT_FALSE(fc.setSetpoint(INFINITY));
    TEST_ASSERT_FALSE(fc.setSetpoint(-1));
    TEST_ASSERT_FALSE(fc.setSetpoint(MAX_FLOW_UL_MIN + 1));
    TEST_ASSERT_EQUAL_FLOAT(0, fc.update(1, 0, true));
    TEST_ASSERT_TRUE(fc.setSetpoint(62.5f));
    TEST_ASSERT_TRUE(fc.isSafe());
}

void test_flow_controller_feedforward_first_update() {
    FlowController fc;
    FlowControlConfig cfg;
    cfg.minCommand = 0;
    cfg.maxCommand = 10;
    fc.configure(cfg);
    ProportionalFlowModel m(25);
    fc.setModel(&m);
    fc.setSetpoint(62.5f);
    // measurement == setpoint → pure feed-forward 2.5
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 2.5f, fc.update(0.01f, 62.5f, true));
}

void test_flow_controller_output_clamped_to_command_limits() {
    FlowController fc;
    FlowControlConfig cfg;
    cfg.minCommand = 1;
    cfg.maxCommand = 3;
    cfg.ki = 1;
    fc.configure(cfg);
    ProportionalFlowModel m(25);
    fc.setModel(&m);
    fc.setSetpoint(70);
    float c = 0;
    for (int i = 0; i < 200; ++i) c = fc.update(0.1f, 0, true);   // no flow at all
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 3.0f, c);
}

void test_flow_controller_runs_at_loop_rate() {
    FlowController fc;
    FlowControlConfig cfg;
    cfg.maxCommand = 10;
    cfg.loopHz = 1;
    cfg.kp = 1;
    cfg.ki = 0;
    fc.configure(cfg);
    ProportionalFlowModel m(10);
    fc.setModel(&m);
    fc.setSetpoint(50);
    const float c0 = fc.update(0.01f, 50, true);
    // Measurement changes but less than 1 s has elapsed → command held.
    const float c1 = fc.update(0.5f, 0, true);
    TEST_ASSERT_EQUAL_FLOAT(c0, c1);
    const float c2 = fc.update(0.5f, 0, true);
    TEST_ASSERT_TRUE(c2 > c1);
}

void test_watchdog() {
    HeartbeatWatchdog w;
    TEST_ASSERT_TRUE(w.expired(0, 10));
    w.feed(1000);
    TEST_ASSERT_FALSE(w.expired(10000, 10));
    TEST_ASSERT_TRUE(w.expired(11500, 10));
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 2.0f, w.ageS(3000));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_pid_proportional_only);
    RUN_TEST(test_pid_integral_clamped);
    RUN_TEST(test_pid_conditional_integration_antiwindup);
    RUN_TEST(test_pid_actuator_saturation_freezes_integrator);
    RUN_TEST(test_flow_controller_rejects_out_of_range);
    RUN_TEST(test_flow_controller_feedforward_first_update);
    RUN_TEST(test_flow_controller_output_clamped_to_command_limits);
    RUN_TEST(test_flow_controller_runs_at_loop_rate);
    RUN_TEST(test_watchdog);
    return UNITY_END();
}
