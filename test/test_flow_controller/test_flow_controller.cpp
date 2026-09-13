// Run with: pio test -e native
// These tests run on your laptop's CPU, not the ESP32 — no hardware needed.
// That's only possible because flow_controller.h/.cpp has no Arduino dependency.

#include <unity.h>
#include "../../src/control/flow_controller.h"

PumpGeometry testGeometry = {
    200.0f,   // steps/rev
    16.0f,    // microstepping
    1.0f,     // mm lead per rev
    50.0f,    // mm^2 syringe area
};

void test_zero_flow_gives_zero_hz() {
    FlowController fc(testGeometry, 0.0f, 50.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fc.computeStepFrequencyHz(0.0f));
    TEST_ASSERT_TRUE(fc.isSafe());
}

void test_above_max_is_rejected_not_clamped() {
    FlowController fc(testGeometry, 0.0f, 50.0f);
    float hz = fc.computeStepFrequencyHz(999.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, hz);
    TEST_ASSERT_FALSE(fc.isSafe());
}

void test_nan_input_is_rejected() {
    FlowController fc(testGeometry, 0.0f, 50.0f);
    float hz = fc.computeStepFrequencyHz(NAN);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, hz);
    TEST_ASSERT_FALSE(fc.isSafe());
}

void test_typical_flow_rate_is_positive_and_safe() {
    FlowController fc(testGeometry, 0.0f, 50.0f);
    float hz = fc.computeStepFrequencyHz(5.0f);
    TEST_ASSERT_TRUE(fc.isSafe());
    TEST_ASSERT_GREATER_THAN(0.0f, hz);
}

void test_clamp_helper_bounds_values() {
    FlowController fc(testGeometry, 0.0f, 50.0f);
    TEST_ASSERT_EQUAL_FLOAT(50.0f, fc.clampFlowRate(999.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fc.clampFlowRate(-5.0f));
    TEST_ASSERT_EQUAL_FLOAT(10.0f, fc.clampFlowRate(10.0f));
}

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;
    UNITY_BEGIN();
    RUN_TEST(test_zero_flow_gives_zero_hz);
    RUN_TEST(test_above_max_is_rejected_not_clamped);
    RUN_TEST(test_nan_input_is_rejected);
    RUN_TEST(test_typical_flow_rate_is_positive_and_safe);
    RUN_TEST(test_clamp_helper_bounds_values);
    return UNITY_END();
}
